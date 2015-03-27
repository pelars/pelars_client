#pragma once
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <string>
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

