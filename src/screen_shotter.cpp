#include "screen_shotter.h"

void screenShotter(const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> & pc){

	synchronizer.lock();

	ScreenGrabber screen_grabber;

	synchronizer.unlock();

	while(!to_stop){

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type = std::string("screen");

		frames->color = screen_grabber.grabScreen();
	
		for(auto & channel : pc){
			channel->write(frames);
		}
	}
	std::cout << "terminating screen shotter" << std::endl;
	
}