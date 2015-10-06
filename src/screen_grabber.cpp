#include "screen_grabber.h"

ScreenGrabber::ScreenGrabber(){
	disp_ = XOpenDisplay(NULL);
    scr_ = DefaultScreen(disp_);
    root_ = DefaultRootWindow(disp_);
}

void ScreenGrabber::grabScreen(){

    surface_ = cairo_xlib_surface_create(disp_, root_, DefaultVisual(disp_, scr_), DisplayWidth(disp_, scr_), DisplayHeight(disp_, scr_));
    cairo_surface_write_to_png(surface_, ("../snapshots/screen" + currentDateTimeNow + ".png").c_str());
    cairo_surface_destroy(surface_);
}