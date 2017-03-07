#include "kinect2publisher.h"
#include "timer_manager.h"

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc, DataWriter & websocket){

	synchronizer.lock();
	
	K2G k2g(processor);
	k2g.disableLog();

	auto mode = K2G::RegistrationMode::Undistorted; 
	auto framemode = ImageFrame::RegistrationMode::Undistorted;
	K2G_Parameters cameraparams(k2g,mode);

	/*
	cv::Mat k2_parameters = cv::Mat::eye(3, 3, CV_32F);
	k2_parameters.at<float>(0,0) = kinect2parameters.fx;
	k2_parameters.at<float>(1,1) = kinect2parameters.fy;
	k2_parameters.at<float>(0,2) = kinect2parameters.cx;
	k2_parameters.at<float>(1,2) = kinect2parameters.cy;
	*/

	CamParams color_params(cameraparams.getKrgb(), cameraparams.getDrgb(), cameraparams.rgb_size.width,cameraparams.rgb_size.height);
	CamParams depth_params(cameraparams.getKir(), cameraparams.getDir(), cameraparams.ir_size.width, cameraparams.ir_size.height); // undistored depth NON registered


	std::cout << "Loaded k2 camera parameters : " << std::endl;
	std::cout << k2_parameters << std::endl;

	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "calibration";
	root["obj"]["camera"] = "kinect2";
	color_params.toJSON(root["obj"]);
	depth_params.toJSON(root["obj"]["depth"]);

	std::string message = writer.write(root);

	if(online){
		io.post( [&websocket, message]() {
		websocket.writeData(message);
		});
	}
	websocket.writeLocal(message);

	synchronizer.unlock();

	TimerManager * tm = TimerManager::instance();
	
	while(!to_stop){


		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type_ = std::string("workspace");
		frames->color_params_ = color_params;
		frames->depth_params_ = depth_params;
		frames->time_stamp_ = std::chrono::high_resolution_clock::now();
		frames->depthmode_ = framemode;

		k2g.get(frames->color_, frames->depth_, mode, false);
		TimerScope ts(tm,"kinectPublisher");
		
		pc.write(frames);
	}
	
	k2g.shutDown();

}