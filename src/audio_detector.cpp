#include "audio_detector.h"

int portAudioCallback(const void * input, void * output, 
					  unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo,
                      PaStreamCallbackFlags statusFlags, void * userData){
	

	FFT * fft = (FFT *)userData;
	if(frameCount > 1){
		float psd = fft->compute((float *)input, frameCount);
	    if(fft->to_send_){
			std::string message = fft->message_;
			if(online){
			        	//std::cout << "Face detector posting data to the server\n " << std::flush;
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

	int used_device = 0;
	for(int i = 0; i < Pa_GetDeviceCount(); ++i)
	    if(std::string(Pa_GetDeviceInfo(i)->name).find("HD Pro Webcam C920") != std::string::npos)
	    	used_device = i;
	
	std::cout << "Using device : " << Pa_GetDeviceInfo(used_device)->name << std::endl;
	
	double srate = 32000;
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