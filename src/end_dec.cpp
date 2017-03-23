#include "enc_dec.hpp"

EncDec::EncDec(bool decoder, int w, int h, int dw, int dh, bool rgba)
{
	if(dw == 0 || dh  == 0)
	{
		dw = w;
		dh = h;
	}
	if(!decoder)
	{
		onfcustom.open("out" + currentDateTime() + ".bin", std::ios::binary);
		onfvideo.open("out" + currentDateTime() + ".mp4", std::ios::binary);
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

bool EncDec::unStep(std::ifstream & inf)
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

void EncDec::step()
{
	step(&rgb_buffer_[0],&depth_buffer_[0]);
}

void EncDec::step(const uint8_t * pc, const uint16_t * pd)
{
	x264_encoder_.encodeFrame((char*)pc, w_*h_*rgb_pixel_size_);

	int rgboutputsize = packRgbData();
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
}

int EncDec::packRgbData()
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