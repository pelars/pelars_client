#include "trigger.h"

void sendTrigger(const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>> & pc_trigger, const int interval){


	std::mutex mutex;
	std::chrono::seconds inter(interval);

	while(!to_stop){

		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
			
		for(auto & t : pc_trigger){
			std::cout << "sending" << std::endl;
			std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>(now, true);
			t->write(trigger);
		}

        auto sleep_time = inter - (std::chrono::high_resolution_clock::now() - now);

        if(sleep_time > std::chrono::microseconds(0))
        {
        	std::unique_lock<std::mutex> lk(mutex);
            timed_stopper.wait_for(lk, sleep_time, [&]{return to_stop;});
            std::cout << "termianted waiting" << std::endl;
        }

	}

}