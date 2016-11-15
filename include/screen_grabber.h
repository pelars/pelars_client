#pragma once
#include <string>
#include <cairo.h>
#ifdef __APPLE__
#else
#include <cairo-xlib.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#endif
#include <iostream>
#include <chrono>
#include <algorithm>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>


class ScreenGrabber{

public:
	
	ScreenGrabber();
	void grabScreen(cv::Mat & in);

private:

    int scr_, width_, height_;
    Display * disp_;
    Window root_;
    XImage * img_;

};

