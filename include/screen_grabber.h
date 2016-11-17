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
#include <memory>
#include <unistd.h>


class ScreenGrabber{

public:
	
	ScreenGrabber();
	cv::Mat grabScreen();
	~ScreenGrabber();

private:

    int width_, height_;
    Display * disp_;
    Window root_;
    XImage * img_;

};

