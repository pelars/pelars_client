#pragma once
#include "xn16zdec.h"
#include "x264encoder.h"
#include "x264decoder.h"

extern const std::string currentDateTimeNow;

template <class T>
bool streamread1(std::istream & ins, T & out)
{
	ins.read((char*)&out,sizeof(out));
	return (bool) ins;
}

template <class T>
bool streamread1(std::istream & ins, std::vector<T> & x, int n)
{
	ins.read((char*)&x[0],sizeof(T)*n);
	return (bool)ins;
}

class EncDec
{
public:
	struct DataPacked
	{
		double time_;
		uint32_t frame_id_;
		uint32_t color_size_;
		uint32_t depth_size_;
		uint16_t w_, h_,dw_,dh_;
	} __attribute__((packed));


	int w_,dw_;
	int h_,dh_;
	int rgb_pixel_size_;
	int dept_pixel_size_;
	int frame_id_ = 0;
	
	std::vector<uint8_t> rgb_buffer_,rgb_buffer_compressed_,depth_buffer_compressed_;
	std::vector<uint16_t> depth_buffer_;
	x264Encoder x264_encoder_;
	x264decoder x264_decoder_;
	std::ofstream onfcustom,onfvideo,onfvideojpeg;

	EncDec(bool decoder, int w, int h, int dw, int dh, bool rgba = false);

	bool unStep(std::ifstream & inf);
	void step();
	void step(const uint8_t * pc, const uint16_t * pd);

private:
	int packRgbData();
};