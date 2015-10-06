#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <data_writer.h>
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include <vector>
#include "opt.h"
#include "gstreamer_grabber.h"

extern bool to_stop;
extern bool visualization;
extern double interval;
extern std::chrono::time_point<std::chrono::system_clock> start;
extern const std::string currentDateTimeNow;
extern bool snapshot;

void detectFaces(DataWriter & websocket);
