#include "kinect2publisher.h"

void kinect2publisher(const K2G::Processor processor, const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> & pc){

	synchronizer.lock();

	K2G k2g(processor);

	synchronizer.unlock();
	
	while(!to_stop){

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type = std::string("workspace");
		k2g.get(frames->color, frames->depth);

		for(auto & channel : pc){
			channel->write(frames);
		}

	}
	
	k2g.shutDown();

}