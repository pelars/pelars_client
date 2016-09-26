#pragma once
#ifdef HAS_ARUCO
#include "k2g.h"
#include "opt.h"
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <boost/filesystem.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "image_sender.h"
#include "base64.h"
#include "x264encoder.h"
#include "gstreamer_grabber.h"
#include "mutex.h"


extern bool to_stop;
extern bool online;
extern bool visualization;
extern double interval;
extern bool snapshot_table;

void handDetector(DataWriter & websocket, float marker_size, ImageSender & image_sender, 
	              K2G::Processor processor, const bool video, const bool store_depth = false, const bool c920 = false, unsigned int camera_id = 1);

#endif