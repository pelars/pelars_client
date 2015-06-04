#pragma once
#include <opencv2/opencv.hpp>

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

	EncDec(bool decoder, int w, int h, int dw, int dh, bool rgba = false)
	{
		if(dw == 0 || dh  == 0)
		{
			dw = w;
			dh = h;
		}
		if(!decoder)
		{
		onfcustom.open("out" + currentDateTimeNow + ".bin",std::ios::binary);
		onfvideo.open("out" + currentDateTimeNow + ".mp4",std::ios::binary);
	}
		rgb_pixel_size_ = rgba ? 4 : 3;
		dept_pixel_size_ = 2;
		w_ = w;
		h_ = h;
		dw_ = dw;
		dh_ = dh;
		x264_encoder_.initialize(w_,h_);
	  	x264_decoder_.initialize(w_,h_);

		rgb_buffer_.resize(w_*h_*rgb_pixel_size_);
		depth_buffer_.resize(dw_*dh_);
		rgb_buffer_compressed_.resize(w_*h_*rgb_pixel_size_);
		depth_buffer_compressed_.resize(dw_*dh_*dept_pixel_size_);
	}

	bool unstep(std::ifstream & inf)
	{
		{
			DataPacked dp;
			if(!streamread1(inf,dp))
				return false;
			if(dp.w_ != w_ || dp.h_ != h_)
			{
				std::cerr << "packet is " << dp.w_ << " " << dp.h_ << std::endl;
				return false;
			}
			streamread1(inf,rgb_buffer_compressed_,dp.color_size_);
			streamread1(inf,depth_buffer_compressed_,dp.depth_size_); // items not bytes
			if(inf)
			{
				if(!x264_decoder_.decodeFrame(&rgb_buffer_compressed_[0],dp.color_size_,&rgb_buffer_[0])) 
				    {
				        std::cerr << "Unable to decode rgb frame\n";
				        return false;
				    }

				    XnUInt32 decompressed_size_bytes = w_*h_*2;
				    if(XnStreamUncompressDepth16Z((XnUInt8 *)&depth_buffer_compressed_[0],
				                                  dp.depth_size_, (XnUInt16 *)&depth_buffer_[0],
				                                  &decompressed_size_bytes) != 0)
				    {
				        std::cerr << "Unable to decode depth frame\n";
				        return false;
				    }			
				   // invoke callback	
			}
			return true;
		}
	}


	void step()
	{
		step(&rgb_buffer_[0],&depth_buffer_[0]);
	}

	void step(const uint8_t * pc, const uint16_t * pd)
	{
		x264_encoder_.encodeFrame((char*)pc, w_*h_*rgb_pixel_size_);

		int rgboutputsize = pack_rgb_data();
		XnUInt32 depthoutputsize = depth_buffer_compressed_.size();
		
		int depthresult = XnStreamCompressDepth16Z((XnUInt16 *)pd,dw_*dh_*2, (XnUInt8 *)&depth_buffer_compressed_[0],
											  &depthoutputsize);

		// use the output as wanted ... 
		onfvideo.write((char*)&rgb_buffer_compressed_[0],rgboutputsize);
		DataPacked dp;
		dp.time_ = 0;
		dp.frame_id_ = frame_id_++;
		dp.color_size_ = rgboutputsize;
		dp.depth_size_ = depthoutputsize;
		dp.w_ = w_;
		dp.h_ = h_;		
		dp.dw_ = dw_;
		dp.dh_ = dh_;		
		onfcustom.write((char*)&dp,sizeof(dp));
		onfcustom.write((char*)&rgb_buffer_compressed_[0],rgboutputsize);
		onfcustom.write((char*)&depth_buffer_compressed_[0],depthoutputsize);

	//	onfvideojpeg
	}

private:
	int pack_rgb_data()
	{
		unsigned tmp_size = 0;
		
		x264_nal_t nal;
		while(x264_encoder_.isNalsAvailableInOutputQueue())
		{
			nal = x264_encoder_.getNalUnit();
		    memcpy(&(rgb_buffer_compressed_[tmp_size]), nal.p_payload, nal.i_payload);
			tmp_size += nal.i_payload;
		}
		return tmp_size;
	}
};