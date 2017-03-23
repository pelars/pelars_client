#include "../include/timer_manager.h"
#include <iostream>
#include <iostream>
#include <chrono>
#include <thread>
 
int main(int argc, char const *argv[])
{
	using namespace std::chrono_literals;
	TimerManager* tm = TimerManager::instance();
	int c = 0;
	while(true)
	{
		std::this_thread::sleep_for(100ms);
		TimerScope scope(tm,"hello");
		std::this_thread::sleep_for(300ms);

		if(++c % 100)
		{
			std::cout << "hello " << tm->timeStatistics("hello").toString() << std::endl;
		}
	}
	return 0;
}