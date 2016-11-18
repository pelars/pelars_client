#include "video_saver.h"

void saveVideo(int session, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc){

	bool inited = false;

	std::string video_folder_name = std::string("../../videos");
	std::string video_subfolder_name = std::string("../../videos/videos_") + std::to_string(session); 

	if(!boost::filesystem::exists(video_folder_name)){
		boost::filesystem::path dir(video_folder_name);
		boost::filesystem::create_directory(video_folder_name);
	}

	if(!boost::filesystem::exists(video_subfolder_name)){
		boost::filesystem::path dir(video_subfolder_name);
		boost::filesystem::create_directory(video_subfolder_name);
	}

	std::shared_ptr<std::ofstream> out;
	std::shared_ptr<x264Encoder> x264encoder;
	std::shared_ptr<ImageFrame> frames;

	while(!to_stop){

		if(pc->readNoWait(frames)){


			if(!inited){
				std::string name = frames->type_ + "_" + time2string(std::chrono::high_resolution_clock::now()) + "_" + std::to_string(session) + ".h264";
				x264encoder = std::make_shared<x264Encoder>(video_subfolder_name, name);
				unsigned int width = frames->color_.cols;
				unsigned int height = frames->color_.rows;
				x264encoder->initialize(width, height, frames->type_ == "people" ? false : true);
				name = video_subfolder_name + "/time_stamps_" + frames->type_ + ".txt";
				out = std::make_shared<std::ofstream>(name, std::ios::binary);

				inited = true;
			}

			if(frames->hasColor()){
				x264encoder->encodeFrame((const char *)(frames->color_).data);
				*out << time2string(frames->time_stamp_) << "\n";
			}
			
		}		
	}
	x264encoder->unInitilize();
}