#include "x264encoder.h"

void x264Encoder::initialize(const unsigned int w, const unsigned int h, const bool kinect)
{
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
	parameters_.rc.i_vbv_buffer_size = 2000000;
	parameters_.rc.i_vbv_max_bitrate = 1500000;
	parameters_.rc.f_rf_constant = 25;
	parameters_.rc.f_rf_constant_max = 35;
	parameters_.i_sps_id = 7;

	time_base_ = 0;
	first_ = true;

	// the following two value you should keep 1
	parameters_.b_repeat_headers = 1;    // to get header before every I-Frame
	parameters_.b_annexb = 1; // put start code in front of nal. we will remove start code later
	x264_param_apply_profile(&parameters_, "high");

	encoder_ = x264_encoder_open(&parameters_);

	picture_in_.i_qpplus1         = 0;
	picture_in_.img.i_plane       = 1;
	picture_in_.i_type = X264_TYPE_AUTO;
	picture_in_.img.i_csp = X264_CSP_I420;
	x264_picture_alloc(&picture_in_, X264_CSP_I420, 
					   parameters_.i_width, parameters_.i_height);
	if(!kinect){
		convert_context_ = sws_getContext(parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_BGR24, 
									  parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_YUV420P,
									  SWS_FAST_BILINEAR, NULL, NULL, NULL);
	}
	else
	{
		convert_context_ = sws_getContext(parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_RGB32, 
									  parameters_.i_width,
									  parameters_.i_height,
									  PIX_FMT_YUV420P,
									  SWS_FAST_BILINEAR, NULL, NULL, NULL);
	}

}

void x264Encoder::unInitilize()
{
	x264_encoder_close(encoder_);
	sws_freeContext(convert_context_);
	os_.close();
	std::string input = folder_name_ + file_name_;
	file_name_.erase (file_name_.end() - 5, file_name_.end());
	std::string mp4_name = file_name_ + std::string(".mp4");
	std::ostringstream strs;
	strs << "ffmpeg -r "  << 1000.0/boost::accumulators::mean(acc_) << " -f h264 -i " << input << " -reset_timestamps 1 -y -c copy -an " << folder_name_ + mp4_name;
	std::string command = strs.str();
	std::cout << "executing " << command << std::endl;
	if(std::system(command.c_str()) == 0)
		std::system((std::string("rm ") + input).c_str());
}

//Encode the rgb frame into a sequence of NALs unit that are stored in a std::vector
void x264Encoder::encodeFrame(const char *rgb_buffer, const unsigned int bytes)
{
	const uint8_t * rgb_buffer_slice[1] = {(const uint8_t *)rgb_buffer};
	int stride[1] = { bytes * image_w_ }; // RGB stride

	//Convert the frame from RGB to YUV420
	int slice_size = sws_scale(convert_context_, rgb_buffer_slice,
							   stride, 0, image_h_, picture_in_.img.plane,
							   picture_in_.img.i_stride);
	x264_nal_t* nals;
	int i_nals = 0;
	int frame_size = -1;
/*
	if(first_){
		picture_out_.i_pts = 0;
		first_ = false;
	}
	else{
		picture_out_.i_pts = time_base_; 
	}
*/

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
			//output_queue_.push(nals[i]);
			
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
