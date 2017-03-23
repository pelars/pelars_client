#include "screen_shotter.h"

void screenShotter(ChannelWrapper<ImageFrame> & pc, unsigned int interval){

	synchronizer.lock();

	ScreenGrabber screen_grabber;
	std::mutex mutex;
	std::chrono::milliseconds inter(interval);

	synchronizer.unlock();

	while(!to_stop){

		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		frames->type_ = std::string("screen");
		frames->color_ = screen_grabber.grabScreen();
	
		pc.write(frames);

		auto sleep_time = inter - (std::chrono::high_resolution_clock::now() - now);

		if(sleep_time > std::chrono::microseconds(0))
		{
			std::unique_lock<std::mutex> lk(mutex);
		    timed_stopper.wait_for(lk, sleep_time, [&]{return to_stop;});
		}
	}
	std::cout << "terminating screen shotter" << std::endl;
	
}