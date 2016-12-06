#include "kinect2publisher.h"

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc, DataWriter & websocket){

	synchronizer.lock();
	
	K2G k2g(processor);
	k2g.disableLog();

	cv::Mat k2_parameters = cv::Mat::eye(3, 3, CV_32F);
	k2_parameters.at<float>(0,0) = kinect2parameters.fx;
	k2_parameters.at<float>(1,1) = kinect2parameters.fy;
	k2_parameters.at<float>(0,2) = kinect2parameters.cx;
	k2_parameters.at<float>(1,2) = kinect2parameters.cy;

	CamParams cam_params(k2_parameters, cv::Mat(), 1920, 1080);

	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "calibration";
	root["obj"]["camera"] = "kinect2";

	Json::Value intrinsics = Json::arrayValue;

	for(size_t i = 0; i < 3; ++i)
		for(size_t j = 0; j < 3; ++j)
			intrinsics.append(k2_parameters.at<float>(i,j));
	root["obj"]["intrinsics"] = intrinsics;

	Json::Value dist = Json::arrayValue;

	for(size_t i = 0; i < 5; ++i)
			dist.append(0.);
	root["obj"]["dist"] = dist;

	std::string message = writer.write(root);

	if(online){
		io.post( [&websocket, message]() {
		websocket.writeData(message);
		});
	}
	websocket.writeLocal(message);

	synchronizer.unlock();
	
	while(!to_stop){

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type_ = std::string("workspace");
		frames->params_ = cam_params;
		frames->time_stamp_ = std::chrono::high_resolution_clock::now();

		k2g.get(frames->color_, frames->depth_);
		cv::flip(frames->color_, frames->color_, 1);
		cv::flip(frames->depth_, frames->depth_, 1);

		pc.write(frames);
	}
	
	k2g.shutDown();

}