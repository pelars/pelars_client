#include "screen_grabber.h"


ScreenGrabber::ScreenGrabber(){

    disp_ = XOpenDisplay(NULL);
    scr_ = DefaultScreen(disp_);
    root_ = DefaultRootWindow(disp_);

    width_ = DisplayWidth(disp_, scr_);
    height_ = DisplayHeight(disp_, scr_);
}

void ScreenGrabber::grabScreen(cv::Mat & in){
#ifdef __APPLE__
#else	

    img_ = XGetImage(disp_, root_, 0, 0, width_, height_, AllPlanes, ZPixmap);
    in = cv::Mat(height_, width_, CV_8UC4, img_->data);

#endif
}

