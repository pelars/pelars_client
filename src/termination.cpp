#include "termination.h"

bool to_stop = false;
std::condition_variable timed_stopper;

// Signal handler
void sig_handler(int signum)
{
		to_stop = true;
		std::cout <<"Received signal " << signum << ", killing all" << std::endl;
		timed_stopper.notify_all();
		std::cout << "HERE 1" << std::endl;
		// Triggers
		for(auto & ch : pc_trigger)
			ch->notify_all();
		for(auto & ch : pc_webcam)
			ch->notify_all();
		for(auto & ch : pc_kinect)
			ch->notify_all();
		for(auto & ch : pc_screen)
			ch->notify_all();

}

void terminateMe(){
	sig_handler(0);
}

