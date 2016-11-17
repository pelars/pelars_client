#include "screen_grabber.h"


ScreenGrabber::ScreenGrabber(){
   
   disp_ = XOpenDisplay(nullptr);
   root_ = DefaultRootWindow(disp_);
   XWindowAttributes attributes = {0};
   XGetWindowAttributes(disp_, root_, &attributes);

   width_ = attributes.width;
   height_ = attributes.height;
   disp_ = XOpenDisplay(nullptr);
   
}

cv::Mat ScreenGrabber::grabScreen(){
#ifdef __APPLE__
#else	

	
    img_ = XGetImage(disp_, root_, 0, 0, width_, height_, AllPlanes, ZPixmap);
    cv::Mat out = cv::Mat(height_, width_, CV_8UC4, img_->data).clone();
    XDestroyImage(img_);

    return out;

#endif
}

ScreenGrabber::~ScreenGrabber(){

	XCloseDisplay(disp_);
}
