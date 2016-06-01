#include "audio_recorder.h"

std::string exec(const char* cmd) {
    char buffer[128];
    std::string result = "";
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer, 128, pipe.get()) != NULL)
            result += buffer;
    }
    return result;
}

void audioRecorder(unsigned int session){

		std::string image_folder_name = std::string("../../audio");

		if(!boost::filesystem::exists(image_folder_name)){
			boost::filesystem::path dir(image_folder_name);
			boost::filesystem::create_directory(dir);
		}

		
		//std::string device = exec("pactl list | grep -A2 'Source #' | grep 'Name: ' | cut -d\" \" -f2 | grep C920");

		//std::cout << "Recording audio from " << device << std::endl;

		std::stringstream ss;

		//ss << "gst-launch-1.0 pulsesrc device=\"" << device << "\" ! audio/x-raw,channels=2 ! avenc_ac3 bitrate=192000 ! filesink location=" << image_folder_name << "/"<< session << ".ac3 &";
		ss << "gst-launch-1.0 alsasrc device=sysdefault:CARD=C920 ! audioconvert ! audioresample ! wavenc ! filesink location=" << image_folder_name << "/"<< session << ".wav &";

		std::string execute = ss.str();
		std::cout << "Executing " << execute << std::endl;
		exec(execute.c_str());
}
