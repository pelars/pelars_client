#include "kinect2publisher.h"

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc){

	synchronizer.lock();

	K2G k2g(processor);

	synchronizer.unlock();
	
	while(!to_stop){

		cv::Mat color, depth;
		k2g.get(color, depth);

		for(auto & ch : pc.getChannels()){
			std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
			frames->type_ = std::string("workspace");
			frames->time_stamp_ = std::chrono::high_resolution_clock::now();
			frames->color_ = color;
			frames->depth_ = depth;
			ch->write(frames);
		}

	}
	
	k2g.shutDown();

}