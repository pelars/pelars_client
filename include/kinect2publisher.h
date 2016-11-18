#pragma once
#include "k2g.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "channel_wrapper.hpp"
#include <memory>
#include "image_frame.h"

extern bool to_stop;
extern std::mutex synchronizer;

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc);