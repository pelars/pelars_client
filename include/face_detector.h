#pragma once
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <data_writer.h>
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include <vector>
#include "opt.h"
#include "x264encoder.h"
#include "termination.h"
#include "image_frame.h"

class DataWriter;

extern bool to_stop;
extern bool visualization;
extern double interval;

void detectFaces(DataWriter & websocket, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pcw, const bool video);
