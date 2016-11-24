#pragma once
#include <iostream>
#include <vector>
#include <fstream>
#include "portaudio.h" 
#include <unsupported/Eigen/FFT>
#include "opt.h"
#include "mutex.h"
#include "lame/lame.h"
#include <boost/filesystem.hpp>
#include <chrono>


extern double interval;
extern bool to_stop;

class DataWriter;

void audioDetector(DataWriter & data_writer);


class Mp3encoder
{
public :
	Mp3encoder(int samplerate, int channels, int bitrate, const std::string & name);
	~Mp3encoder();
	void encode_inter(short * s, int samples);
	void flush();

	std::vector<char> buf_;
	std::ofstream onf_;
	lame_global_flags * parameters_;
};

struct FFT{

	float psd_;
	float scale_;
	std::vector<float> amplitude_;
	std::vector<std::complex<float>> freqvec_;
	Eigen::FFT<float> fft_;
	DataWriter & websocket_;
	Json::Value root_;
  	Json::StyledWriter writer_;
  	std::string message_;
  	TimedSender timer_;
  	Mp3encoder mp3encoder_;

	FFT(DataWriter & websocket, int samplerate, int channels, int bitrate, const std::string & name);

	void compute(float * buf, unsigned int count);
};

