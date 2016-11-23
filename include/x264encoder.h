#pragma once
#ifdef __cplusplus
#define __STDINT_MACROS
#define __STDC_CONSTANT_MACROS
#endif
#include <iostream>
#include <queue>
#include <stdint.h>

#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
	#include "x264.h"
	#include <libswscale/swscale.h>
}

#include <fstream>
#include <chrono>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <sstream>
#include <stdlib.h>

class x264Encoder
{

public:
	x264Encoder(const std::string folder_name = "./", const std::string file_name = "video.mp4"): 
	            file_name_(file_name), folder_name_(folder_name){}

	void initialize(const unsigned int w, const unsigned int h, const bool kinect, const bool del);
	void unInitilize();
	void encodeFrame(const char *rgb_buffer);
	bool isNalsAvailableInOutputQueue();
	int image_h_, image_w_;

	x264_nal_t getNalUnit();
	x264_t * getx264Encoder() { return encoder_; }
	int nal_size() { return output_queue_.size(); }

private:

	SwsContext * convert_context_ = nullptr;
	std::queue<x264_nal_t> output_queue_;
	x264_param_t parameters_;
	x264_picture_t picture_in_, picture_out_;
	x264_t * encoder_;
	std::ofstream os_;
	long int time_base_;
	bool first_, delete_;
	int bytes_;
	std::string file_name_, folder_name_;
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean>> acc_;
};

