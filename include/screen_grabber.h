#pragma once
#include <string>
#include <chrono>
#include <algorithm>

class ScreenGrabber{

public:
	
	ScreenGrabber();
	void grabScreen(std::string name);

private:

    int scr_;

};

