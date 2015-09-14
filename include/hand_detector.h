#pragma once
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>        
#include <opencv2/features2d/features2d.hpp>
//#include "kinect_grabber.h"
//#include "kinect2_grabber.hpp"
#include "opt.h"
#include "data_writer.h"
#include <iostream>
#include "alttime.h"
#include <json/json.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "gstreamer_grabber2.h"
#include "gstreamer_grabber.h"
#include <ctime>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/packet_pipeline.h>

extern bool to_stop;
extern bool online;
extern bool visualization;

void handDetector(DataWriter & websocket, int session);