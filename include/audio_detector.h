#pragma once
#include "portaudio.h" 
#include <unsupported/Eigen/FFT>
#include "opt.h"

extern bool to_stop;
extern double interval;

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
  	TimedSender timer_;

	FFT(DataWriter & websocket): websocket_(websocket), timer_(interval / 2.0){
		root_["obj"]["type"] = "audio";
	}

	void compute(float * buf, unsigned int count);
};