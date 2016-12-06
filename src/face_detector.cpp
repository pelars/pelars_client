#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <data_writer.h>
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include <vector>
#include "opt.h"
#include "x264encoder.h"
#include "termination.h"
#include "image_frame.h"
#include "face_detector.h"

// Returns the distance in meters
inline double distance(int x1, int x2, float fx, float std_width = 185.0){
		return ((std_width * fx) / std::abs(x1 - x2)) / 1000;
	}

#ifdef HAS_GSTREAMER
void detectFaces(DataWriter & websocket, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pcw, const bool video)
{

	synchronizer.lock();
	cv::Mat calib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	cv::FileStorage file("../../data/calibration_webcam.xml", cv::FileStorage::READ);
	if(file.isOpened())
	{	
		file["matrix"] >> calib_matrix;
		file.release();
	}else{
		std::cout << "could not find face calibration file; use -c to calibrate the cameras" << std::endl;
		to_stop = true;
	}

	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	std::string face_cascade_name_gpu_("../../data/haarcascade_frontalface_alt2.xml");
	cv::CascadeClassifier face_cascade_;

	cv::gpu::GpuMat gray_gpu;
	cv::Mat faces_downloaded, color, cam_matrix, dist;
	std::shared_ptr<ImageFrame> color_frame;

	const int session = websocket.getSession();

	const bool findLargestObject_ = false;
	const bool filterRects_ = true;
	bool inited = false;

	cv::gpu::CascadeClassifier_GPU cascade_gpu_;
	cascade_gpu_.visualizeInPlace = false;
	cascade_gpu_.findLargestObject = findLargestObject_;

	std::string video_folder_name = std::string("../../videos");
	std::string video_subfolder_name = std::string("../../videos/videos_") + std::to_string(session); 
	
	Json::Value upper;
	Json::Value root = Json::arrayValue;
	Json::Value array;

	float face_distance, tx, ty, tz, tx1, ty1, tz1, tx2, ty2, tz2, x_unproject, y_unproject, fx, fy, cx, cy;
	cv::Mat pose;
	cv::Mat face_pose = cv::Mat(cv::Size(1, 4), CV_32F);
	face_pose.at<float>(0, 3) = 1;
	int detections_num;
	unsigned int width, height;

	// Preapare JSON message to send to the Collectorh
	std::string code;

	Json::StyledWriter writer;

	if(!cascade_gpu_.load(face_cascade_name_gpu_))
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_gpu_ << std::endl; 
		to_stop = true;
	}
	
	if(visualization)
		cv::namedWindow("face");

	TimedSender timer(interval);

	cv::Mat camera_inverse = calib_matrix.inv();
	
	synchronizer.unlock();

	while(!to_stop)
	{	

		cv::gpu::GpuMat facesBuf_gpu;
		if(!pcw->read(color_frame))
			continue;

		if(!inited){
			auto params = color_frame->params_;
			cam_matrix = params.cam_matrix_;
			fx = cam_matrix.at<float>(0,0);
			fy = cam_matrix.at<float>(1,1);
			cx = cam_matrix.at<float>(0,2);
			cy = cam_matrix.at<float>(1,2);
			dist = params.dist_;
			width = params.width_;
			height = params.height_;
			inited = true;
		}

		// too slow
		//cv::undistort(color_frame->color_, color, cam_matrix, dist);
		color = color_frame->color_;

		//cv::flip(color, color, 1);

		cv::gpu::GpuMat color_gpu(color);

		cv::gpu::cvtColor(color_gpu, gray_gpu, CV_BGR2GRAY);
		
		detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, cv::Size(width, height), cv::Size(), 
														1.05, (filterRects_ || findLargestObject_) ? 4 : 0);

		facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
		cv::Rect * faces = faces_downloaded.ptr<cv::Rect>();

		
		//std::cout << " found " << detections_num << " faces" << std::endl;
		// Take same time for all faces
		std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
		array["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
		for(int i = 0; i < detections_num; ++i)
		{

			cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
			cv::ellipse(color, center, cv::Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, cv::Scalar(255, 0, 255), 4, 8, 0);

			// Distance in meters
			face_distance = distance(faces[i].x, faces[i].x + faces[i].width, fx);

			const float const_x = (face_distance / fx);
			const float const_y = (face_distance / fy);

			x_unproject = (faces[i].x - cx) * const_x;
			y_unproject = (faces[i].y - cy) * const_y;

			//std::cout << x_unproject << " " << y_unproject << std::endl;
				
			const float tmp = y_unproject;
			
			//face_pose.at<float>(0, 3) = 1; already done after declaration
			face_pose.at<float>(0, 0) = x_unproject;
			face_pose.at<float>(0, 1) = y_unproject;
			face_pose.at<float>(0, 2) = face_distance;

			pose = camera_inverse * face_pose;
			tx = pose.at<float>(0, 0);
			ty = pose.at<float>(0, 1);
			tz = pose.at<float>(0, 2);

			x_unproject = (faces[i].x + faces[i].width - cx) * const_x;
			y_unproject = (faces[i].y + faces[i].height - cy) * const_y;

			face_pose.at<float>(0, 0) = x_unproject;
			face_pose.at<float>(0, 1) = y_unproject;

			pose = camera_inverse * face_pose;
			tx1 = pose.at<float>(0, 0);
			ty1 = pose.at<float>(0, 1);
			tz1 = pose.at<float>(0, 2);

			// Reuse previous values
			face_pose.at<float>(0, 0) = x_unproject;
			face_pose.at<float>(0, 1) = tmp;

			pose = camera_inverse * face_pose;
			tx2 = pose.at<float>(0, 0);
			ty2 = pose.at<float>(0, 1);
			tz2 = pose.at<float>(0, 2);

			// Json message
			array["type"] = "face";
			array["id"] = i;
			array["x"] = tx; 
			array["y"] = -ty;
			array["z"] = tz;
			array["x1"] = tx1;
			array["y1"] = -ty1;
			array["z1"] = tz1;
			array["x2"] = tx2;
			array["y2"] = -ty2;
			array["z2"] = tz2;
			array["distance"] = face_distance;
			
			cv::circle(color, cv::Point(faces[i].x, faces[i].y), 10, cv::Scalar(0,0,255), -1);
			cv::circle(color, cv::Point(faces[i].x + faces[i].height, faces[i].y + faces[i].width), 10, cv::Scalar(0,255,0), -1);
			cv::circle(color, cv::Point(faces[i].x + faces[i].height, faces[i].y ), 10, cv::Scalar(255,0,0), -1);
			cv::circle(color, cv::Point(faces[i].x, faces[i].y + faces[i].width ), 10, cv::Scalar(255,255,0), -1);

			root.append(array);
		}
		
		upper["obj"] = root;
		// Send message
		if(detections_num && timer.needSend()){
			std::string out_string = writer.write(upper);
			if(online){
				io.post( [&websocket, out_string]() {
				websocket.writeData(out_string);
				});
			}
			websocket.writeLocal(out_string);  
		}

		//gray_gpu.release();
		//facesBuf_gpu.release();
		upper.clear();
		root.clear();
		array.clear();

		// Show what you got
		if(visualization){
			cv::imshow("face", color);
			int c = cv::waitKey(30);
			if((char)c == 'q' ) {
				terminateMe();
				std::cout << "stop requested by face detector" << std::endl;
			}
		}
	}
/*
	if(video)
		x264encoder->unInitilize();
		*/
	if(visualization)
		cvDestroyWindow("face");

	std::cout << "terminating face detector" << std::endl;
}

#else
void detectFaces(DataWriter & websocket, ScreenGrabber & screen_grabber, ImageSender & image_sender_screen, 
	             ImageSender & image_sender_people, const int face_camera_id, const bool video)
{
}
#endif
