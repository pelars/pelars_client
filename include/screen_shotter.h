#pragma once
#include "trigger.h"
#include "image_frame.h"
#include "screen_grabber.h"
#include "channel_wrapper.hpp"


extern std::mutex synchronizer;

void screenShotter(ChannelWrapper<ImageFrame> & pc, unsigned int interval);