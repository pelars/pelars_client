#include "x264encoder.h"

#ifndef PIX_FMT_RGB32
#define PIX_FMT_RGB32 AV_PIX_FMT_RGB32
#define PIX_FMT_BGR24 AV_PIX_FMT_BGR24
#define PIX_FMT_YUV420P AV_PIX_FMT_YUV420P
#endif

void x264Encoder::initialize(const unsigned int w, const unsigned int h, const bool kinect, const bool del)
{

	delete_ = del;
	folder_name_ = folder_name_.back() == '/' ? folder_name_ : folder_name_ + "/";
	os_.open(folder_name_ + file_name_, std::ios::binary);

	image_w_ = w;
	image_h_ = h;
	x264_param_default_preset(&parameters_, "veryfast", "zerolatency");

	parameters_.i_log_level = X264_LOG_INFO;
	parameters_.i_threads = 1;
	parameters_.i_width = image_w_;
	parameters_.i_height = image_h_;
	parameters_.i_fps_num = 30;
	parameters_.i_fps_den = 1;
	parameters_.i_keyint_max = 25;
	parameters_.b_intra_refresh = 1;
	parameters_.rc.i_rc_method = X264_RC_CRF;
	parameters_.rc.i_vbv_buffer_size = 300000;
	parameters_.rc.i_vbv_max_bitrate = 300000;
	parameters_.rc.f_rf_constant = 25;
	parameters_.rc.f_rf_constant_max = 35;
	parameters_.i_sps_id = 7;

	time_base_ = 0;
	first_ = true;

	parameters_.b_repeat_headers = 1;   
	parameters_.b_annexb = 1;
	x264_param_apply_profile(&parameters_, "high");

	encoder_ = x264_encoder_open(&parameters_);


	picture_in_.i_qpplus1         = 0;
	picture_in_.img.i_plane       = 1;
	picture_in_.i_type = X264_TYPE_AUTO;
	picture_in_.img.i_csp = X264_CSP_I420;
	x264_picture_alloc(&picture_in_, X264_CSP_I420, 
					   parameters_.i_width, parameters_.i_height);

	if(kinect){
		std::cout << "Recording Kinect2" << std::endl;
		convert_context_ = sws_getContext(parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_RGB32, 
									  parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_YUV420P,
									  SWS_FAST_BILINEAR, NULL, NULL, NULL);
		bytes_ = 4;
	}
	else
	{
		std::cout << "Recording Webcam" << std::endl;
		convert_context_ = sws_getContext(parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_BGR24, 
									  parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_YUV420P,
									  SWS_FAST_BILINEAR, NULL, NULL, NULL);
		bytes_ = 3;
	}

}

void x264Encoder::unInitilize()
{
	x264_encoder_close(encoder_);
	sws_freeContext(convert_context_);
	os_.close();
	std::string input = folder_name_ + file_name_;
	file_name_.erase (file_name_.end() - 4, file_name_.end());
	std::cout << "FILE NAME " << file_name_ << std::endl;
	std::string mp4_name = file_name_ + std::string("mp4");
	std::ostringstream strs;
	strs << "ffmpeg -r "  << 1000.0/boost::accumulators::mean(acc_) << " -f h264 -i " << input << " -reset_timestamps 1 -y -c copy -an " << folder_name_ + mp4_name;
	std::string command = strs.str();
	std::cout << "Executing " << command << std::endl;
	if(std::system(command.c_str()) == 0){
		std::cout << "h264 converted to mp4 successfully" << std::endl;
		if(delete_ && std::system((std::string("rm ") + input).c_str()) == 0)
			std::cout << "h264 file removed" << std::endl;
		else
			std::cout << "Could not remove " << input << std::endl; 
	}
}

void x264Encoder::encodeFrame(const char *rgb_buffer)
{
	const uint8_t * rgb_buffer_slice[1] = {(const uint8_t *)rgb_buffer};
	int stride[1] = { (int)bytes_ * image_w_ }; // RGB stride

	//Convert the frame from RGB to YUV420
	int slice_size = sws_scale(convert_context_, rgb_buffer_slice,
							   stride, 0, image_h_, picture_in_.img.plane,
							   picture_in_.img.i_stride);
	x264_nal_t* nals;
	int i_nals = 0;
	int frame_size = -1;

	std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
	long int interval = (long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count() - time_base_;
	time_base_ = (long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
	frame_size = x264_encoder_encode(encoder_, &nals, &i_nals,
									 &picture_in_, &picture_out_);
	if(!first_)
		acc_(interval);

	if(first_)
		first_ = false;

	if(frame_size > 0)
		for(int i = 0; i< i_nals; i++){
			os_.write((const char*)nals[i].p_payload, nals[i].i_payload);
		}
}

bool x264Encoder::isNalsAvailableInOutputQueue()
{
	if(output_queue_.empty() == true)
		return false;
	else
		return true;
}

x264_nal_t x264Encoder::getNalUnit()
{
	x264_nal_t nal;
	nal = output_queue_.front();
	output_queue_.pop();
	return nal;
}
