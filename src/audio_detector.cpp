#include "audio_detector.h"
#include <iostream>
#include <vector>
#include <fstream>
#include "portaudio.h" 
#include <unsupported/Eigen/FFT>
#include "opt.h"
#include "mutex.h"
#include "lame/lame.h"


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

Mp3encoder::Mp3encoder(int samplerate, int channels, int bitrate, const std::string & name): 
                       onf(name.c_str(), std::ios::binary)
{
	buf.resize(1024*64);
	p = lame_init();
	if(!p)
	{
		std::cout << "bad lame initialization" << std::endl;
		exit(0);
	}
	lame_set_in_samplerate(p, samplerate); // default is 44100
	lame_set_num_channels(p, channels);
	lame_set_out_samplerate(p, 0); // automatic
/*
	  internal algorithm selection.  True quality is determined by the bitrate
	  but this variable will effect quality by selecting expensive or cheap algorithms.
	  quality=0..9.  0=best (very slow).  9=worst.
	  recommended:  2     near-best quality, not too slow
	                5     good quality, fast
	                7     ok quality, really fast
*/
	//lame_set_quality(p,...)
	lame_set_brate(p, bitrate);

	lame_init_params(p);
}

void Mp3encoder::encode_inter(short * s, int samples)
{
	int n = lame_encode_buffer_interleaved(p, s, samples,(unsigned char*)&buf[0], buf.size());
	onf.write(&buf[0], n);
}

void Mp3encoder::flush()
{
	int n = lame_encode_flush(p,(unsigned char*)&buf[0],buf.size());
	onf.write(&buf[0], n);
}

Mp3encoder::~Mp3encoder()
{
	lame_close(p);
}

int portAudioCallback(const void * input, void * output, 
					  unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo,
					  PaStreamCallbackFlags statusFlags, void * userData){


	FFT * fft = (FFT *)userData;
	if(frameCount > 1 ){
		fft->compute((float *)input, frameCount);
		if(fft->timer_.needSend()){
			const std::string message = fft->message_;
			//std::cout << fft->psd_ << std::endl;
			if(online){
				io.post([&fft, message]() {
					fft->websocket_.writeData(message);
				});
			}
			fft->websocket_.writeLocal(message);  
		}
	}
	fft->mp3encoder.encode_inter((short*)input, frameCount);

	return 0;
}

void audioDetector(DataWriter & data_writer){
	
	Pa_Initialize();

	int used_device = -1;
	for(int i = 0; i < Pa_GetDeviceCount(); ++i){
		//std::cout << Pa_GetDeviceInfo(i)->name << std::endl;
		//if(std::string(Pa_GetDeviceInfo(i)->name).find("HD Pro Webcam C920") != std::string::npos)
			//used_device = i;
		if(std::string(Pa_GetDeviceInfo(i)->name).find("Xbox NUI Sensor") != std::string::npos)
			used_device = i;
	}

	if(used_device == -1){
		std::cout << "Xbox NUI Sensor" << std::endl;
		Pa_Terminate();
		return;
	}
	
	std::cout << "Using device : " << Pa_GetDeviceInfo(used_device)->name << std::endl;
	
	const double srate = Pa_GetDeviceInfo(used_device)->defaultSampleRate;
	std::cout << "Sampling @" << srate << "Hz" << std::endl;
	PaStream * stream;
	unsigned long framesPerBuffer = paFramesPerBufferUnspecified; 
	PaStreamParameters inputParameters;
	inputParameters.channelCount = Pa_GetDeviceInfo(used_device)->maxInputChannels;
	std::cout << "Device has " << inputParameters.channelCount << " channels" << std::endl;
	inputParameters.device = used_device;
	inputParameters.hostApiSpecificStreamInfo = nullptr;
	inputParameters.sampleFormat =  paInt32;//paFloat32;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(used_device)->defaultLowInputLatency;

	FFT fft(data_writer, srate, inputParameters.channelCount, srate, "test.mp3");
	if(Pa_OpenStream(&stream, &inputParameters, nullptr, srate, framesPerBuffer, paNoFlag, portAudioCallback, &fft) || Pa_StartStream(stream)){
		std::cout << "error opening stream. Audio won't be available" << std::endl;
	} else{
		while(!to_stop)
			Pa_Sleep(3000);
	}
		
	Pa_CloseStream(stream);
	Pa_Terminate();
}

void FFT::compute(float * buf, unsigned int count){
		
	psd_ = 0;
	scale_ = 1 / (float)count;

	if(amplitude_.size() < count){
		amplitude_.resize(count);
		freqvec_.resize(count);
	}

	fft_.fwd(&freqvec_[0], &buf[0], count);

	for(unsigned int i = 0; i < count; ++i){
		amplitude_[i] = 2 * scale_ * sqrt(pow(freqvec_[i].real(), 2) + pow(freqvec_[i].imag(), 2));
		psd_ += pow(amplitude_[i], 2) * scale_;
	}
	
	root_["obj"]["value"] = psd_;
	std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
	root_["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
	message_ = writer_.write(root_);
}
