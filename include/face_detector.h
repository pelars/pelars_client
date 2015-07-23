#pragma once
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/features2d/features2d.hpp>

#include <data_writer.h>
#include <boost/network/protocol/http/client.hpp>
#include "kinect_grabber.h"
#include "opencv2/gpu/gpu.hpp"
#include "opencv2/core/gpumat.hpp"
#include <json/json.h>
#include "alttime.h"
#include <vector>

extern bool to_stop;

void detectFaces(DataWriter & websocket, int session);
void detectFacesCPU(DataWriter & websocket, int session);
