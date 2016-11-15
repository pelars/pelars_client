#include "trigger.h"

void sendTrigger(const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>> & pc_trigger, int interval){

	while(!to_stop){

		std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
		std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>(now, true);

		for(auto & t : pc_trigger){
			t->write(trigger);
		}

		//VERGOGNOSO MA TEST
		for(int i = 0; i < interval; ++i){
			if(to_stop){
				break;
			}
			sleep(1);
		}
	}

}