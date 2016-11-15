#pragma once
#include "trigger.h"
#include "pooledchannel.hpp"
#include "image_frame.h"
#include "screen_grabber.h"


extern std::mutex synchronizer;

void screenShotter(const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> & pc);