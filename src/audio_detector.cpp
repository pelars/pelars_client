#include "audio_detector.h"

const float threshold = 0.0005;

int portAudioCallback(const void * input, void * output, 
					  unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo,
					  PaStreamCallbackFlags statusFlags, void * userData){
	FFT * fft = (FFT *)userData;
	if(frameCount > 1 ){
		const float psd = fft->compute((float *)input, frameCount);
		if(psd > threshold && fft->timer_.needSend()){
			const std::string message = fft->message_;
			//std::cout << psd << std::endl;
			if(online){
				io.post([&fft, message]() {
					fft->websocket_.writeData(message);
				});
			}
			fft->websocket_.writeLocal(message);  
		}
	}
	return 0;
}

void audioDetector(DataWriter & data_writer){

	Pa_Initialize();

	int used_device = -1;
	for(int i = 0; i < Pa_GetDeviceCount(); ++i)
		if(std::string(Pa_GetDeviceInfo(i)->name).find("HD Pro Webcam C920") != std::string::npos)
			used_device = i;

	if(used_device == -1){
		std::cout << "no C920 found" << std::endl;
		return;
	}
	std::cout << "Using device : " << Pa_GetDeviceInfo(used_device)->name << std::endl;
	
	const double srate = 32000;
	PaStream * stream;
	unsigned long framesPerBuffer = paFramesPerBufferUnspecified; 
	PaStreamParameters outputParameters;
	PaStreamParameters inputParameters;
	inputParameters.channelCount = Pa_GetDeviceInfo(used_device)->maxInputChannels;
	inputParameters.device = used_device;
	inputParameters.hostApiSpecificStreamInfo = NULL;
	inputParameters.sampleFormat = paFloat32;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(used_device)->defaultLowInputLatency;
	inputParameters.hostApiSpecificStreamInfo = NULL; 

	FFT fft(data_writer);
	if(Pa_OpenStream(&stream, &inputParameters, NULL, srate, framesPerBuffer, paNoFlag, portAudioCallback, &fft) || Pa_StartStream(stream)){
		std::cout << "error opening stream. Audio won't be available" << std::endl;
	} else{
		while(!to_stop)
			Pa_Sleep(10000);
	}
		
	Pa_CloseStream(stream);
	Pa_Terminate();
}

float FFT::compute(float * buf, int count){
		
	psd_ = 0;
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
	std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
	root_["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
	message_ = writer_.write(root_);

	return psd_;
}