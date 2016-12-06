#pragma once
#include "k2g.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "channel_wrapper.hpp"
#include <memory>
#include "image_frame.h"
#include "param_storage.h"

extern bool to_stop;
extern std::mutex synchronizer;
extern ParameterSpace parameters;
extern libfreenect2::Freenect2Device::ColorCameraParams kinect2parameters;

void kinect2publisher(const K2G::Processor processor, ChannelWrapper<ImageFrame> & pc);