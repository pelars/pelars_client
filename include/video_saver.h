#pragma once
#include "x264encoder.h"
#include <fstream>
#include <memory>
#include "pooledchannel.hpp"
#include "image_frame.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include "opt.h"

extern bool to_stop;

void saveVideo(int session, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc, bool del);