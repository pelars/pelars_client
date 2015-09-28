#include "audio_detector.h"

int portAudioCallback(const void * input, void * output, 
					  unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo,
                      PaStreamCallbackFlags statusFlags, void * userData){
	
	FFT * fft = (FFT *)userData;
	float psd = fft->compute((float *)input, frameCount);
	if(online){
	        	//std::cout << "Face detector posting data to the server\n " << std::flush;
          		io.post( [&fft]() {
                	fft->websocket_.writeData(fft->message_);
                });
            }
            fft->websocket_.writeLocal(fft->message_);  
	
	if(psd > 0.02)
		std::cout << psd << std::endl;

    return 0;

}
/*
int portAudioCallback(const void * input, void * output, 
					  unsigned long frameCount, const PaStreamCallbackTimeInfo * timeInfo,
                      PaStreamCallbackFlags statusFlags, void * userData ){
	
	float * buf = (float*)input;
	float psd = 0;
	const float scale = 1 / frameCount;

	std::vector<float> in(frameCount);
	std::vector<float> amplitude(frameCount);
	std::vector<std::complex<float>> freqvec(frameCount);

	Eigen::FFT<float> fft;
	
	for(int i = 0; i < frameCount; ++i)
		in.push_back(buf[i]);

	fft.fwd(freqvec, in);
	
    for(int i = 0; i < frameCount; ++i){
    	amplitude[i] = 2 * scale * sqrt(freqvec[i].real()*freqvec[i].real()  + freqvec[i].imag()*freqvec[i].imag());
    	psd += amplitude[i] * amplitude[i] * scale;
    }

	if(psd > 0.01)
		std::cout << "HIGH" << std::endl;
    return 0;

}*/

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