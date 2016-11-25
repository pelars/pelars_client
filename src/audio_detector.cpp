#include "audio_detector.h"



FFT::FFT(DataWriter & websocket, int samplerate, int channels, int bitrate, const std::string & name): 
         websocket_(websocket), timer_(interval / 2), mp3encoder_(samplerate, channels, bitrate, name)
{
	root_["obj"]["type"] = "audio";
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

	psd_ = 20 * log10(psd_);
	
	root_["obj"]["value"] = psd_;
	std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
	root_["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
	message_ = writer_.write(root_);
}

Mp3encoder::Mp3encoder(int samplerate, int channels, int bitrate, const std::string & name): 
                       onf_(name.c_str(), std::ios::binary)
{
	buf_.resize(1024*64);
	parameters_ = lame_init();
	if(!parameters_)
	{
		std::cout << "bad lame initialization" << std::endl;
		exit(0);
	}
	lame_set_in_samplerate(parameters_, samplerate); // default is 44100
	lame_set_num_channels(parameters_, channels);
	lame_set_out_samplerate(parameters_, 0); // automatic
/*
	  internal algorithm selection.  True quality is determined by the bitrate
	  but this variable will effect quality by selecting expensive or cheap algorithms.
	  quality=0..9.  0=best (very slow).  9=worst.
	  recommended:  2     near-best quality, not too slow
	                5     good quality, fast
	                7     ok quality, really fast
*/
	//lame_set_quality(p,...)
	lame_set_brate(parameters_, bitrate);
	lame_init_params(parameters_);
}

void Mp3encoder::encode_inter(short * s, int samples)
{
	int n = lame_encode_buffer_interleaved(parameters_, s, samples,(unsigned char*)&buf_[0], buf_.size());
	onf_.write(&buf_[0], n);
}

void Mp3encoder::flush()
{
	int n = lame_encode_flush(parameters_,(unsigned char*)&buf_[0], buf_.size());
	onf_.write(&buf_[0], n);
}

Mp3encoder::~Mp3encoder()
{
	lame_close(parameters_);
}

int portAudioCallback(const void * input, void * output, 
					  unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo,
					  PaStreamCallbackFlags statusFlags, void * userData){


	FFT * fft = (FFT *)userData;
	
	if(frameCount > 1 ){
		uint16_t * in = (uint16_t *)input;
		// Cast data frm unint_16 to float
		fft->float_cast_buf_.clear();
		for(unsigned int i = 0; i < frameCount; ++i){
			fft->float_cast_buf_.push_back(static_cast<float>(in[i]));
		}

		fft->compute(&(fft->float_cast_buf_[0]), frameCount);
		if(fft->timer_.needSend()){
			const std::string message = fft->message_;
			std::cout << fft->psd_ << std::endl;
			if(online && !isinf(fft->psd_)){
				io.post([&fft, message]() {
					fft->websocket_.writeData(message);
				});
			}
			fft->websocket_.writeLocal(message);  
		}
	}
	fft->mp3encoder_.encode_inter((short*)input, frameCount);

	return 0;
}

void audioDetector(DataWriter & data_writer){
	
	Pa_Initialize();
	//Get audio info pactl list

	int used_device = -1;
	for(int i = 0; i < Pa_GetDeviceCount(); ++i){
		if(std::string(Pa_GetDeviceInfo(i)->name).find("HD Pro Webcam C920") != std::string::npos)
			used_device = i;
		//if(std::string(Pa_GetDeviceInfo(i)->name).find("Xbox NUI Sensor") != std::string::npos)
			//used_device = i;
	}

	if(used_device == -1){
		std::cout << "No C920 Connected" << std::endl;
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
	inputParameters.sampleFormat = paInt16;//paFloat32;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(used_device)->defaultLowInputLatency;

	std::string audio_folder_name = std::string("../../audio/");
	std::string audio_name = "audio_" + time2string(std::chrono::high_resolution_clock::now()) + "_" + std::to_string(data_writer.getSession()) + ".mp3";
  

	if(!boost::filesystem::exists(audio_folder_name)){
		boost::filesystem::path dir(audio_folder_name);
		boost::filesystem::create_directory(audio_folder_name);
	}

	FFT fft(data_writer, srate, inputParameters.channelCount, srate, audio_folder_name + audio_name);
	if(Pa_OpenStream(&stream, &inputParameters, nullptr, srate, framesPerBuffer, paNoFlag, portAudioCallback, &fft) || Pa_StartStream(stream)){
		std::cout << "error opening stream. Audio won't be available" << std::endl;
	} else{
		while(!to_stop)
			Pa_Sleep(3000);
	}
		
	Pa_CloseStream(stream);
	Pa_Terminate();
	fft.mp3encoder_.flush();
}

