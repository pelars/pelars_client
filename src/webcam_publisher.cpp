#include "webcam_publisher.h"


void webcamPublisher(int face_camera_id, ChannelWrapper<ImageFrame> & pc_webcam, unsigned int width, unsigned int height){

	synchronizer.lock();

	GstreamerGrabber gs_grabber(width, height, face_camera_id);

	cv::Mat c920_parameters, k;
	cv::FileStorage in("../../data/c920_parameters.xml", cv::FileStorage::READ);
	if(width == 1920 && height == 1080){
		in["cameraMatrix1920x1080"] >> c920_parameters;
		in["distortionMatrix1920x1080"] >> k;
	}
	else if(width == 800 && height == 448){
		in["cameraMatrix800x448"] >> c920_parameters;
		in["distortionMatrix800x448"] >> k;
	}
	else{
		std::cout << "no correct calibration for c920 found" << std::endl;
		terminateMe();
	}

	CamParams cam_params(c920_parameters, k, width, height);

	synchronizer.unlock();
	
	while(!to_stop){

		IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

		gs_grabber.capture(frame);

		std::shared_ptr<ImageFrame> image = std::make_shared<ImageFrame>();
		image->color_ = cv::Mat(frame);
		image->type_ = std::string("people");
		image->params_ = cam_params;
		image->time_stamp_ = std::chrono::high_resolution_clock::now();

		pc_webcam.write(image);
		
	}
	std::cout << "terminating webcam publisher" << std::endl;

}