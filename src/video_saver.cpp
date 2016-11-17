#include "video_saver.h"

void saveVideo(int session, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc){

	bool inited = false;
	std::chrono::high_resolution_clock::time_point p;

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
	std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();

	while(!to_stop){

		if(pc->readNoWait(frames)){

			p = std::chrono::high_resolution_clock::now();

			if(!inited){
				std::string name = video_subfolder_name + "/" + frames->type + "_" + time2string(p) + "_" + std::to_string(session) + ".h264";
				x264encoder = std::make_shared<x264Encoder>(name);
				unsigned int width = frames->color.cols;
				unsigned int height = frames->color.rows;
				x264encoder->initialize(width, height, frames->type == "people" ? false : true);
				name = video_subfolder_name + "/time_stamps_" + frames->type + ".txt";
				out = std::make_shared<std::ofstream>(name);

				inited = true;
			}

			if(frames->color.rows > 0){
				x264encoder->encodeFrame((const char *)(frames->color).data);
				*out << time2string(p) << "\n";
			}
			
		}		
	}
	x264encoder->unInitilize();
}