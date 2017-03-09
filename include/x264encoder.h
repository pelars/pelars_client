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
#include "param_storage.h"

extern ParameterSpace parameters;


class x264Encoder
{

public:

	x264Encoder(std::ofstream * onf) : onf_(onf) {}	
	void initialize(const unsigned int w, const unsigned int h, const bool isBGR, const bool hasAlpha, const int threads = 1);	
	void unInitilize();	
	void encodeFrame(const char *rgb_buffer);	
	bool isNalsAvailableInOutputQueue();	
	int image_h_, image_w_;	x264_nal_t getNalUnit();	
	x264_t * getx264Encoder() { return encoder_; }	
	int nal_size() { return output_queue_.size(); }	
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean>> getAcc(){ return acc_; }

private:

	SwsContext * convert_context_ = nullptr;
	std::queue<x264_nal_t> output_queue_;
	x264_param_t parameters_;
	x264_picture_t picture_in_, picture_out_;
	x264_t * encoder_;
	std::ofstream os_;
	std::ofstream * onf_;
	long int time_base_;
	bool first_, delete_;
	int bytes_;
	std::string file_name_, folder_name_;
	boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::mean>> acc_;
};
