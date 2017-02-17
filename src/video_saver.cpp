#include "video_saver.h"

void saveVideo(int session, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc, bool del){

	bool inited_color = false;
	bool inited_depth = false;

	std::string video_folder_name = std::string("../../videos");
	std::string video_subfolder_name = std::string("../../videos/videos_") + std::to_string(session); 

	if(!boost::filesystem::exists(video_subfolder_name)){
		boost::filesystem::path dir(video_subfolder_name);
		boost::filesystem::create_directories(video_subfolder_name);
	}

	std::shared_ptr<std::ofstream> out;
	std::shared_ptr<std::ofstream> out_oni;
	std::shared_ptr<x264Encoder> x264encoder;
	std::shared_ptr<ImageFrame> frames;

	cv::Mat depth(cv::Size(512, 424), CV_32FC1);
	cv::Mat depth_16(cv::Size(512, 424), CV_16U);
	XnUInt8 * depth_8 = new XnUInt8[512*424*2];
	XnUInt32 depth_8_size;

	long old_seq = 0;
	long new_seq = 0;
	bool kinect = false;

	while(!to_stop){

		if(pc->read(frames)){

			if(frames->hasColor()){


				if(!inited_color){
					std::string name = frames->type_ + "_" + time2string(std::chrono::high_resolution_clock::now()) + "_" + std::to_string(session) + ".h264";
					x264encoder = std::make_shared<x264Encoder>(video_subfolder_name, name);
					unsigned int width = frames->color_.cols;
					unsigned int height = frames->color_.rows;
					x264encoder->initialize(width, height, frames->type_ == "people" ? false : true, del);
					name = video_subfolder_name + "/time_stamps_" + frames->type_ + ".txt";
					out = std::make_shared<std::ofstream>(name, std::ios::binary);
					old_seq = frames->seq_number_;
					inited_color = true;
				}

				new_seq = frames->seq_number_;

				//if(new_seq != 0)
				//	std::cout << new_seq << " " << old_seq << std::endl;

				if(new_seq - old_seq > 1 && old_seq != 0){
					std::cout << "!!!frame lost!!!" << std::endl;
					if(frames->hasDepth()){
						std::cout << "from Kinect" << std::endl;
					}
					else{
						std::cout << "from webcam" << std::endl;
					}
				}

				old_seq = new_seq;

				x264encoder->encodeFrame((const char *)(frames->color_).data);
			}

			if(frames->hasDepth()){

				kinect = true;

				if(!inited_depth){
					std::string file_name =  "depth_" + time2string(std::chrono::high_resolution_clock::now()) + "_" + std::to_string(session) + ".oni";
					out_oni = std::make_shared<std::ofstream>(video_subfolder_name + std::string("/") + file_name, std::ios::binary);
					inited_depth = true;

				}

				frames->depth_.convertTo(depth_16, CV_16U);
				XnStreamCompressDepth16Z((const XnUInt16*)depth_16.data, 512*424*2, depth_8, &depth_8_size);
				*out_oni << depth_8_size;
				out_oni->write((const char*)depth_8, depth_8_size);
			}

			*out << time2string(frames->time_stamp_) << "\n";
		}		
	}
	x264encoder->unInitilize();
}