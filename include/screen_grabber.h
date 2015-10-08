#pragma once
#include <cairo.h>
#include <cairo-xlib.h>
#include <X11/Xlib.h>
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

