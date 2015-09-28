#pragma once
#include "portaudio.h" 
#include <iostream>
#include <unsupported/Eigen/FFT>
#include "opt.h"
#include "alttime.h"

extern bool to_stop;

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

	FFT(DataWriter & websocket): websocket_(websocket){
		root_["obj"]["type"] = "audio";
	}

	float compute(float * buf, int count){
		psd_ = 0;
		scale_ = 1 / count;

		if(in_.size() < count && count != 0){
			amplitude_.resize(count);
			freqvec_.resize(count);
		}
		in_.resize(count + 1);

		for(int i = 0; i < count; ++i){
			//in_[i] = 0;
			in_[i] = buf[i];
		}
		//in_.assign(buf, buf + count);

		fft_.fwd(freqvec_, in_);
		
	    for(int i = 0; i < count; ++i){
	    	amplitude_[i] = 2 * scale_ * sqrt(pow(freqvec_[i].real(), 2) + pow(freqvec_[i].imag(), 2));
	    	psd_ += pow(amplitude_[i], 2) * scale_;
	    }

	    root_["obj"]["value"] =  psd_;
	    root_["obj"]["value"] = deltats(orwl_gettime(), start);
	    message_ = writer_.write(root_);

		return psd_;
	}
};