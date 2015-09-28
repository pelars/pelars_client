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
  	double elapsed_, begin_;
  	bool to_send_;


	FFT(DataWriter & websocket): websocket_(websocket){
		root_["obj"]["type"] = "audio";
		elapsed_ = 0;
		begin_ = clock();
		to_send_ = false;
	}

	float compute(float * buf, int count){
		to_send_ = false;
		psd_ = 0;
		elapsed_ = float(clock() - begin_) / CLOCKS_PER_SEC;
		if(elapsed_ > 0.01){
			scale_ = 1 / (float)count;

			if(amplitude_.size() < count){
				amplitude_.resize(count);
				freqvec_.resize(count);
			}

			fft_.fwd(&freqvec_[0], &buf[0], count);

		    for(int i = 0; i < count; ++i){
		    	amplitude_[i] = 2 * scale_ * sqrt(pow(freqvec_[i].real(), 2) + pow(freqvec_[i].imag(), 2));
		    	psd_ += pow(amplitude_[i], 2) * scale_;
		    }
		    
		    root_["obj"]["value"] = psd_;
		    root_["obj"]["time"] = deltats(orwl_gettime(), start);
		
		    message_ = writer_.write(root_);
		    if(psd_ > 0.001){
		    	elapsed_ = 0.0;
		    	begin_ = clock();
		    	to_send_ = true;
		    }
		}
	
		return psd_;
	}
};