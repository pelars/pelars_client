
#ifndef X264ENCODER_H
#define X264ENCODER_H

#ifdef __cplusplus
#define __STDINT_MACROS
#define __STDC_CONSTANT_MACROS
#endif
#include <iostream>
#include <queue>
#include <stdint.h>

#ifndef INT64_C
#define INT64_C(c) (c ## LL)
#define UINT64_C(c) (c ## ULL)
#endif

extern "C" {
	#include "x264.h"
	#include <libswscale/swscale.h>
}

#include <fstream>

class x264Encoder
{

public:
	x264Encoder(const std::string name = "video.x264"): os_(name, std::ios::binary) {}
	void initialize(int w, int h, const bool kinect = false);
	void unInitilize();
	void encodeFrame(const char *rgb_buffer, int buffer_size);
	bool isNalsAvailableInOutputQueue();
	int image_h_;
	int image_w_;

	x264_nal_t getNalUnit();
	x264_t *getx264Encoder() { return encoder_; }
	int nal_size() { return output_queue_.size(); }
private:
	// Use this context to convert your BGR Image to YUV image since x264 do not support RGB input
	SwsContext* convert_context_ = NULL;
	std::queue<x264_nal_t> output_queue_;
	x264_param_t parameters_;
	x264_picture_t picture_in_, picture_out_;
	x264_t* encoder_;
	std::ofstream os_;
};

#endif // X264ENCODER_H
