#pragma once
#include <opencv2/core/core.hpp>  

#if defined(HAS_ARUCO) && defined(HAS_FREENECT2)
#include "k2g.h"

void calibration(const unsigned int face_camera_id, const unsigned int hand_camera_id, const float marker_size, bool c920, 
	             K2G::Processor processor = K2G::OPENCL, bool force = false);

#endif


void store(const cv::Mat & kcalib_matrix, const cv::Mat & wcalib_matrix);
