#include "termination.h"

// Signal handler
void sig_handler(int signum)
{
		to_stop = true;
		std::cout <<"Received signal " << signum << ", killing all" << std::endl;
}

void terminateMe(){
	kill(0, SIGUSR1);
}

struct sigaction new_action;
struct sigaction old_action;

void initTermination(){
	/* Set up the structure to specify the new action. */
	new_action.sa_handler = sig_handler;
	sigemptyset(&new_action.sa_mask);
	new_action.sa_flags = 0;       
	sigaction(SIGUSR1, &new_action, NULL);	
}

bool to_stop = false;