#include "trigger.h"

void sendTrigger(ChannelWrapper<Trigger> & pc_trigger, const int interval){


	std::mutex mutex;
	std::chrono::seconds inter(interval);

	while(!to_stop){

		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
		std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>(now, true);
			
		pc_trigger.write(trigger);

        auto sleep_time = inter - (std::chrono::high_resolution_clock::now() - now);

        if(sleep_time > std::chrono::microseconds(0))
        {
        	std::unique_lock<std::mutex> lk(mutex);
            timed_stopper.wait_for(lk, sleep_time, [&]{return to_stop;});
        }

	}

}