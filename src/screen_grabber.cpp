#include "screen_grabber.h"
#include <cairo.h>
#ifdef __APPLE__
#else
#include <cairo-xlib.h>
#include <X11/Xlib.h>
#endif
#include <iostream>
#include <chrono>
#include <algorithm>

ScreenGrabber::ScreenGrabber(){}

void ScreenGrabber::grabScreen(std::string name){
#ifdef __APPLE__
#else	
    Display * disp_;
    Window root_;
    cairo_surface_t * surface_;

    disp_ = XOpenDisplay(NULL);
    scr_ = DefaultScreen(disp_);
    root_ = DefaultRootWindow(disp_);

    surface_ = cairo_xlib_surface_create(disp_, root_, DefaultVisual(disp_, scr_), DisplayWidth(disp_, scr_), DisplayHeight(disp_, scr_));
    cairo_surface_write_to_png(surface_, name.c_str());
    cairo_surface_destroy(surface_);
#endif
}