#pragma once
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>        
#include <opencv2/features2d/features2d.hpp>
#include "kinect_grabber.h"
#include "data_writer.h"
#include <iostream>
#include "alttime.h"
#include <json/json.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "gstreamer_grabber.h"
#include <ctime>

extern bool to_stop;
extern bool online;
extern bool visualization;

void handDetector(DataWriter & websocket, int session);