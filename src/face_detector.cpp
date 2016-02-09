#include "face_detector.h"

double std_width = 185.0; //mm

double focal_length_pixel = 589.3588305153235; //489.3;  //pixel

inline double distance(int x1, int x2){
		return (std_width * focal_length_pixel) / std::abs(x1 - x2);
	}

void detectFaces(DataWriter & websocket, ScreenGrabber & screen_grabber, ImageSender & image_sender_screen, ImageSender & image_sender_people, const int face_camera_id)
{
	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	//std::string face_cascade_name_ = "../../data/haarcascade_frontalface_alt.xml";
	std::string face_cascade_name_gpu_ = "../../data/haarcascade_frontalface_alt2.xml";
	cv::CascadeClassifier face_cascade_;
	cv::Mat gray;

	const int session = websocket.getSession();

	cv::gpu::CascadeClassifier_GPU cascade_gpu_;
	const bool findLargestObject_ = false;
	const bool filterRects_ = true;

	const int width = 800.0;
	const int height = 448.0;

	const float fx = 589.3588305153235;
	const float cx = 414.1871817694326;
	const float fy = 588.585116717914;
	const float cy = 230.3588624031242; 

	/*
	const float k1 = 0.12269303;
	const float k2 = -0.26618881;
	const float p1 = 0.00129035;
	const float p2 = 0.00081791;
	const float k3 = 0.17005303;
	*/


	float face_distance, tx, ty, tz, tx1, ty1, tx2, ty2, x_unproject, y_unproject;
	cv::Mat pose;
	cv::Mat hand_pose = cv::Mat(cv::Size(1, 4), CV_32F);
	hand_pose.at<float>(0, 3) = 1;

	// Preapare JSON message to send to the Collectorh
	std::string code;

	cv::Mat calib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	Json::StyledWriter writer;

	GstreamerGrabber gs_grabber(width, height, face_camera_id);
	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3); 

	if(!cascade_gpu_.load(face_cascade_name_gpu_))
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_gpu_ << std::endl; 
		to_stop = true;
	}
	
	if(visualization)
		cv::namedWindow("face");

	TimedSender timer(interval);

	cv::FileStorage file("../../data/calibration_webcam.xml", cv::FileStorage::READ);
	if(file.isOpened())
	{	
		file["matrix"] >> calib_matrix;
		file.release();
	}else{
		std::cout << "could not find face calibration file; use -c to calibrate the cameras" << std::endl;
		to_stop = true;
	}

	cv::Mat camera_inverse = calib_matrix.inv();

	std::string folder_name = std::string("../../images/snapshots_") + std::to_string(session);
	while(!to_stop)
	{	
		gs_grabber.capture(frame);
		cv::Mat color(frame);
		cv::flip(color, color, 1);
		cvtColor(color, gray, CV_BGR2GRAY);

		if(snapshot_people && image_sender_people){
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			if(!boost::filesystem::exists(folder_name)){
				boost::filesystem::path dir(folder_name);
				boost::filesystem::create_directory(dir);
			}
			std::string name = std::string(folder_name + "/people_" + now + "_" + std::to_string(session) +".jpg");
			std::cout << name << std::endl;
			cv::imwrite(name, color);
			if(online){
				std::ifstream in(name, std::ifstream::binary);
				in.unsetf(std::ios::skipws);
			    in.seekg(0, std::ios::end);
			    std::streampos fileSize = in.tellg();
			    in.seekg(0, std::ios::beg);
			    std::vector<char> data(fileSize);
				in.read(&data[0], fileSize);
				std::string code = base64_encode((unsigned char*)&data[0], (unsigned int)data.size());
				image_sender_people.send(code, "jpg");
			}
			snapshot_people = false;
		}
		if(snapshot_screen && image_sender_screen){
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			std::string name = std::string(folder_name + "/screen_" + now + "_" + std::to_string(session) + ".png");
			
			if(!boost::filesystem::exists(folder_name)){
				boost::filesystem::path dir(folder_name);
				boost::filesystem::create_directory(dir);
			}
			screen_grabber.grabScreen(name);
			if(online){
				std::ifstream in(name, std::ifstream::binary);
				in.unsetf(std::ios::skipws);
			    in.seekg(0, std::ios::end);
			    std::streampos fileSize = in.tellg();
			    in.seekg(0, std::ios::beg);
			    std::vector<char> data(fileSize);
				in.read(&data[0], fileSize);
				std::string code = base64_encode((unsigned char*)&data[0], (unsigned int)data.size());
				image_sender_screen.send(code, "png");
			}
			snapshot_screen = false;
		}
	
		int detections_num;
		cv::Mat faces_downloaded;
		cv::gpu::GpuMat facesBuf_gpu;
		cv::Mat im(gray.size(), CV_8UC1);
	
		gray.copyTo(im);
		
		cv::gpu::GpuMat gray_gpu(im);

		cascade_gpu_.visualizeInPlace = false;
		cascade_gpu_.findLargestObject = findLargestObject_;
		detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, cv::Size(gray.cols,gray.rows), cv::Size(), 1.05, (filterRects_ || findLargestObject_) ? 4 : 0);

		facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
		cv::Rect * faces = faces_downloaded.ptr<cv::Rect>();

		Json::Value upper;
		Json::Value root = Json::arrayValue;
		Json::Value array;
		//std::cout << " found " << detections_num << " faces" << std::endl;
		for(int i = 0; i < detections_num; ++i)
		{

			cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
			cv::ellipse(gray, center, cv::Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, cv::Scalar( 255, 0, 255), 4, 8, 0);

			face_distance = distance(faces[i].x, faces[i].x + faces[i].width);

			const float const_x = face_distance / fx;
			const float const_y = face_distance / fy;

			x_unproject = (faces[i].x - cx) * const_x;
			y_unproject = (faces[i].y - cy) * const_y;

			//std::cout << x_unproject << " " << y_unproject << std::endl;
				
			const float tmp = y_unproject;
			
			//hand_pose.at<float>(0, 3) = 1; already done after declaration
			hand_pose.at<float>(0, 0) = x_unproject;
			hand_pose.at<float>(0, 1) = y_unproject;
			hand_pose.at<float>(0, 2) = face_distance;

			pose = camera_inverse * hand_pose;
			tx = pose.at<float>(0, 0);
			ty = pose.at<float>(0, 1);
			tz = pose.at<float>(0, 2);

			x_unproject = (faces[i].x + faces[i].width - cx) * const_x;
			y_unproject = (faces[i].y + faces[i].height - cy) * const_y;

			hand_pose.at<float>(0, 0) = x_unproject;
			hand_pose.at<float>(0, 1) = y_unproject;

			pose = camera_inverse * hand_pose;
			tx1 = pose.at<float>(0, 0);
			ty1 = pose.at<float>(0, 1);

			// Reuse previous values
			hand_pose.at<float>(0, 0) = x_unproject;
			hand_pose.at<float>(0, 1) = tmp;

			pose = camera_inverse * hand_pose;
			tx2 = pose.at<float>(0, 0);
			ty2 = pose.at<float>(0, 1);

			// Json message
			array["type"] = "face";
			array["id"] = i;
			array["x"] = tx; 
			array["y"] = ty;
			array["x1"] = tx1;
			array["y1"] = ty1;
			array["x2"] = tx2;
			array["y2"] = ty2;

			//std::cout << "x " << tx << " y " << ty << ", x1 " << tx1 << " y1 " << ty1 << ", x2  " << tx2 << " y2 " << ty2 << ", tz " << tz <<  " distance " << face_distance << std::endl;

			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			array["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
			array["distance"] = tz;
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

		gray_gpu.release();
		facesBuf_gpu.release();

		// Show what you got
		if(visualization){
			cv::imshow( "face", color);
			int c = cv::waitKey(1);
			if((char)c == 'q' ) {
				to_stop = true;
				std::cout << "stop requested by face detector" << std::endl;
			}
		}
	}
}

