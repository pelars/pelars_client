#pragma once
#include "timer_manager.h"
#include <thread>

struct timedThread{

	std::thread thread;
	TimerManager * tm;

	timedThread(TimerManager *, const std::string&);

};