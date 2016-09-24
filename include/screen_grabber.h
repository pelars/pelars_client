#pragma once
#include <string>

class ScreenGrabber{

public:
	
	ScreenGrabber();
	void grabScreen(std::string name);

private:

    int scr_;

};

