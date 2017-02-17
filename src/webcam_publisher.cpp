#include "webcam_publisher.h"


void webcamPublisher(int face_camera_id, ChannelWrapper<ImageFrame> & pc_webcam, unsigned int width, unsigned int height, DataWriter & websocket){

	synchronizer.lock();

	GstreamerGrabber gs_grabber(width, height, face_camera_id);

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

	synchronizer.unlock();

	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	long sequence = 0;
	
	while(!to_stop){

		gs_grabber.capture(frame);

		std::shared_ptr<ImageFrame> image = std::make_shared<ImageFrame>();
		image->color_ = cv::Mat(frame);
		image->type_ = std::string("people");
		image->params_ = cam_params;
		image->time_stamp_ = std::chrono::high_resolution_clock::now();
		image->seq_number_ = sequence;

		sequence ++;

		pc_webcam.write(image);
		
	}
	std::cout << "terminating webcam publisher" << std::endl;

}