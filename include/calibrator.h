#pragma once
#ifdef HAS_ARUCO
#include "k2g.h"
#include "opt.h"
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <boost/filesystem.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "gstreamer_grabber.h"

void calibration(const unsigned int id, const float marker_size);
#else
inline void calibration(const unsigned int id, const float marker_size)
{
	
}

#endif