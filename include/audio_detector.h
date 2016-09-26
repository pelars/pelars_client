#pragma once
#include "portaudio.h" 
#include <unsupported/Eigen/FFT>
#include "opt.h"
#include "mutex.h"
#include "lame/lame.h"

extern bool to_stop;
extern double interval;

void audioDetector(DataWriter & data_writer);

class Mp3encoder
{
public :
	Mp3encoder(int samplerate, int channels, int bitrate, const std::string & name);
	~Mp3encoder();
	void encode_inter(short * s, int samples);

	void flush();

	std::vector<char> buf;
	std::ofstream onf;
	lame_global_flags * p;
};

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
  	Mp3encoder mp3encoder;

	FFT(DataWriter & websocket, int samplerate, int channels, int bitrate, const std::string & name): 
	    websocket_(websocket), timer_(interval / 2.0), 
	    mp3encoder(samplerate, channels, bitrate, name)
	{
		root_["obj"]["type"] = "audio";
	}

	void compute(float * buf, unsigned int count);
};