#include "kinect2publisher.h"

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc){

	synchronizer.lock();

	K2G k2g(processor);
	k2g.disableLog();

	synchronizer.unlock();
	
	while(!to_stop){

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type_ = std::string("workspace");
		frames->time_stamp_ = std::chrono::high_resolution_clock::now();
		k2g.get(frames->color_, frames->depth_);

		pc.write(frames);

	}
	
	k2g.shutDown();

}