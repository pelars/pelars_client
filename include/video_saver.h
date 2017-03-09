#pragma once
#include "x264encoder.h"
#include <fstream>
#include <memory>
#include "pooledchannel.hpp"
#include "image_frame.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include "opt.h"
#include "oni_compressor.h"
#include <string>

extern bool to_stop;

void saveVideo(int session, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc, bool del, const std::string, const int threads);

void x264convertmp4(const std::string name_h264file, const std::string name_mp4file, bool del, x264Encoder & x264encoder);