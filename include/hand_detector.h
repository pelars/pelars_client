#pragma once
#ifdef HAS_ARUCO
#include "kinect2publisher.h"
#include "opt.h"
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "x264encoder.h"
#include "gstreamer_grabber.h"
#include "mutex.h"
#include "termination.h"
#include "image_frame.h"

extern bool to_stop;
extern bool online;
extern bool visualization;
extern double interval;

extern libfreenect2::Freenect2Device::ColorCameraParams kinect2parameters;

void handDetector(DataWriter & websocket, float marker_size, 
	              std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc, 
	              const bool c920 = false, unsigned int camera_id = 1);

#endif