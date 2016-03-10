#pragma once
#include "opt.h"
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include "gstreamer_grabber.h"
#include <opencv2/imgproc/imgproc.hpp>

void show_markers(const unsigned int id, const float marker_size);