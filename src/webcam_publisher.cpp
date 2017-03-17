#include "webcam_publisher.h"
#include "timer_manager.h"
#include <unistd.h>


//TODO:try to open a gstreamer grabber that publishes h263 stream to network
//gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! h264parse ! rtph264pay ! udpsink  host=127.0.0.1 port=5100

void webcamPublisher(int face_camera_id, ChannelWrapper<ImageFrame> & pc_webcam, ChannelWrapper<ImageFrame> & pc_w_saver, unsigned int width, unsigned int height, DataWriter & websocket){

	synchronizer.lock();

	#ifndef OPENCV_CAP
	#ifdef OLD_GSTREAMER
	GstreamerGrabber gs_grabber(width, height, face_camera_id);
	#else
		//GstreamerGrabber gs_grabber(width, height, face_camera_id,true,"v4l2src device=/dev/video0 ! h264parse ! rtph264pay ,clock-rate=(int)90000 ! queue ! udpsink  name=sink host=127.0.0.1 port=5100");
		GstreamerGrabber gs_grabber(width,height,face_camera_id,true,0);
	#endif
	#endif

	cv::Mat c920_parameters, k;

	if(!boost::filesystem::exists("../../data/c920_parameters.xml")){
		std::cout << "ERROR: ../../data/c920_parameters.xml does not exist" << std::endl;
		terminateMe();
		return;
	}

	cv::FileStorage in("../../data/c920_parameters.xml", cv::FileStorage::READ);
	if(width == 1920 && height == 1080){
		in["cameraMatrix1920x1080"] >> c920_parameters;
		in["distCoeff1920x1080"] >> k;
	}
	else if(width == 800 && height == 448){
		in["cameraMatrix800x448"] >> c920_parameters;
		in["distCoeff800x448"] >> k;
	}
	else{
		std::cout << "no correct calibration for c920 found" << std::endl;
		terminateMe();
		return;
	}

	std::cout << "Loaded webcam camera parameters : " << std::endl;
	std::cout << c920_parameters << std::endl;

	std::cout << "Loaded webcam distortion parameters : " << std::endl;
	std::cout << k << std::endl;

	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["camera"] = "webcam";
	root["obj"]["type"] = "calibration";
	Json::Value intrinsics = Json::arrayValue;

	for(size_t i = 0; i < 3; ++i)
		for(size_t j = 0; j < 3; ++j)
			intrinsics.append(c920_parameters.at<float>(i,j));
	root["obj"]["intrinsics"] = intrinsics;

	Json::Value dist = Json::arrayValue;
	for(size_t i = 0; i < 5; ++i){
		dist.append(k.at<float>(0,i));
	}
	root["obj"]["dist"] = dist;


	std::string message = writer.write(root);

	if(online){
		io.post( [&websocket, message]() {
		websocket.writeData(message);
		});
	}
	websocket.writeLocal(message);

	CamParams cam_params(c920_parameters, k, width, height);

	long sequence = 0;

	#ifdef OPENCV_CAP
	cv::VideoCapture cap;

	std::cout << "Capture opening" << std::endl;

	if(!cap.open(face_camera_id)){
	 	to_stop = true;
	}
	
	std::cout << "Capture opened" << std::endl;

	//cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
	//cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
	cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
    cap.set(CV_CAP_PROP_FPS, 30);
    #else
    IplImage * frame_for_face = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
    IplImage * frame_for_save = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	#endif

    #ifdef MONITOR
	TimerManager * tm = TimerManager::instance();
	#endif

	synchronizer.unlock();
	
	//cv::namedWindow("prova");
	
	while(!to_stop){

		std::shared_ptr<ImageFrame> image_for_face = std::make_shared<ImageFrame>();
		std::shared_ptr<ImageFrame> image_for_save = std::make_shared<ImageFrame>();

		#ifdef OPENCV_CAP
		cap >> image_for_face->color_;
		#else
		//gs_grabber >> frame_for_save;
		gs_grabber >> frame_for_face;
		image_for_face->color_ = cv::Mat(frame_for_face);
		//image_for_save->color_ = cv::Mat(frame_for_save);
		#endif

		#ifdef MONITOR
			TimerScope ts(tm,"webcamPublisher");
		#endif

		image_for_face->type_ = std::string("people");
		image_for_face->color_params_ = cam_params;
		image_for_face->time_stamp_ = std::chrono::high_resolution_clock::now();
		image_for_face->seq_number_ = sequence;
		/*image_for_save->type_ = std::string("people");
		image_for_save->params_ = cam_params;
		image_for_save->time_stamp_ = std::chrono::high_resolution_clock::now();
		image_for_save->seq_number_ = sequence;*/

		sequence ++;

		pc_webcam.write(image_for_face);
		pc_w_saver.write(image_for_face);

		//cv::imshow("prova", image_for_face->color_);
		//cv::waitKey(1);

		//cvReleaseImage(&frame);
	}

	std::cout << "terminating webcam publisher" << std::endl;

}