#include "audio_detector.h"

int portAudioCallback(const void *input, void *output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo,
                      PaStreamCallbackFlags statusFlags, void *userData ){

	std::cout << "callback" << std::endl;

}

void audioDetector(){

	Pa_Initialize();

	int used_device = 0;
	for(int i = 0; i < Pa_GetDeviceCount(); ++i)
	    if(std::string(Pa_GetDeviceInfo(i)->name).find("HD Pro Webcam C920: USB Audio") != std::string::npos)
	    	used_device = i;
	
	std::cout << "Using device : " << Pa_GetDeviceInfo(used_device)->name << std::endl;
	
	double srate = 32000;
	PaStream *stream;
	unsigned long framesPerBuffer = paFramesPerBufferUnspecified; 
	PaStreamParameters outputParameters;
	PaStreamParameters inputParameters;
	inputParameters.channelCount = Pa_GetDeviceInfo(used_device)->maxInputChannels;
	inputParameters.device = used_device;
	inputParameters.hostApiSpecificStreamInfo = NULL;
	inputParameters.sampleFormat = paFloat32;
	inputParameters.suggestedLatency = Pa_GetDeviceInfo(used_device)->defaultLowInputLatency ;
	inputParameters.hostApiSpecificStreamInfo = NULL; 
	
	Pa_OpenStream(&stream, &inputParameters, NULL, srate, framesPerBuffer, paNoFlag, portAudioCallback, NULL);

	Pa_StartStream(stream);

	while(!to_stop){
		Pa_Sleep(100);
	}
}