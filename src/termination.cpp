#include "termination.h"

bool to_stop = false;
std::condition_variable timed_stopper;

// Signal handler
void sig_handler(int signum)
{
		to_stop = true;
		std::cout <<"Received signal " << signum << ", killing all" << std::endl;
		timed_stopper.notify_all();
		// Triggers
		
		pc_trigger.shutDown();
		pc_webcam.shutDown();
		pc_kinect.shutDown();
		pc_screen.shutDown();

}

void terminateMe(){
	sig_handler(0);
}

