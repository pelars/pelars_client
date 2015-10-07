#pragma once
#include <cairo.h>
#include <cairo-xlib.h>
#include <X11/Xlib.h>
#include <string>
#include <chrono>

class ScreenGrabber{

public:
	
	ScreenGrabber();
	void grabScreen();

private:

	Display * disp_;
    Window root_;
    cairo_surface_t * surface_;
    int scr_;
};
