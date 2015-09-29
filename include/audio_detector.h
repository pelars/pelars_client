#pragma once
#include "portaudio.h" 
#include <iostream>
#include <unsupported/Eigen/FFT>
#include "opt.h"

extern bool to_stop;
extern double interval;
extern std::chrono::time_point<std::chrono::system_clock> start;

void audioDetector(DataWriter & data_writer);

struct FFT{

	float psd_;
	float scale_;
	std::vector<float> in_, amplitude_;
	std::vector<std::complex<float>> freqvec_;
	Eigen::FFT<float> fft_;
	DataWriter & websocket_;
	Json::Value root_;
  	Json::StyledWriter writer_;
  	std::string message_;
  	TimedSender timer_ = {interval};

	FFT(DataWriter & websocket): websocket_(websocket){
		root_["obj"]["type"] = "audio";
	}

	float compute(float * buf, int count);
};