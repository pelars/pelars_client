#include "audio_detector.h"

int portAudioCallback(const void *input, void *output, unsigned long frameCount, const PaStreamCallbackTimeInfo* timeInfo,
                      PaStreamCallbackFlags statusFlags, void *userData ){

	std::cout << "callback" << std::endl;

}

AudioDetector::AudioDetector(){

	Pa_Initialize();

	int used_device = 0;
	for(int i = 0; i < Pa_GetDeviceCount(); ++i)
	    if(std::string(Pa_GetDeviceInfo( i )->name).find("HD Pro Webcam C920: USB Audio") != std::string::npos)
	    	used_device = i;
	
	std::cout << "DEVICE " << used_device << std::endl;
	
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
	inputParameters.hostApiSpecificStreamInfo = NULL; //See you specific host's API docs for info on using this field
	
	Pa_OpenStream(
	                &stream,
	                &inputParameters,
	                NULL,
	                srate,
	                framesPerBuffer,
	                paNoFlag, //flags that can be used to define dither, clip settings and more
	                (portAudioCallback), //your callback function
	                (void *)this );
/*
	Pa_OpenDefaultStream(
                &stream,
                1,
                1,
                srate,
                framesPerBuffer,
                paNoFlag, //flags that can be used to define dither, clip settings and more
                (portAudioCallback), //your callback function
                (void *)this );*/

Pa_StartStream( stream );

}