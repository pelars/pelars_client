#pragma once
#include "k2g.h"
#include <opencv2/imgproc/imgproc.hpp>
#include "pooledchannel.hpp"
#include <memory>
#include "image_frame.h"

extern bool to_stop;
extern std::mutex synchronizer;

void kinect2publisher(const K2G::Processor processor, const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> & pc);