#pragma once
#include "k2g.h"
#include "opt.h"
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

extern bool to_stop;
extern bool online;
extern bool visualization;
extern double interval;
extern std::chrono::time_point<std::chrono::system_clock> start;

void handDetector(DataWriter & websocket, float marker_size);

