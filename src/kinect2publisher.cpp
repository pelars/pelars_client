#include "kinect2publisher.h"

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc){

	synchronizer.lock();
	
	K2G k2g(processor);
	k2g.disableLog();

	cv::Mat k2_parameters = cv::Mat::eye(3, 3, CV_32F);
	k2_parameters.at<float>(0,0) = kinect2parameters.fx;
	k2_parameters.at<float>(1,1) = kinect2parameters.fy;
	k2_parameters.at<float>(0,2) = kinect2parameters.cx;
	k2_parameters.at<float>(1,2) = kinect2parameters.cy;

	CamParams cam_params(k2_parameters, cv::Mat(), 1920, 1080);

	synchronizer.unlock();
	
	while(!to_stop){

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type_ = std::string("workspace");
		frames->params_ = cam_params;
		frames->time_stamp_ = std::chrono::high_resolution_clock::now();
		k2g.get(frames->color_, frames->depth_);

		pc.write(frames);

	}
	
	k2g.shutDown();

}