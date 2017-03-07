#pragma once
#ifdef HAS_FREENECT2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <thread>
#include <vector>
#include <signal.h>
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <limits>
#include <libfreenect2/logger.h>
#include "serialization.h"

extern bool to_stop;

class K2G {

public:
	enum class Registration {
		None, Undistorted, ColorToDepth, DepthToColor
	};

	enum Processor{
		CPU, OPENCL, OPENGL
	};

	K2G(Processor p);
	libfreenect2::Freenect2Device::IrCameraParams getIrParameters();
	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();
	void shutDown();
	cv::Mat getColor();
	void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);
	void get(cv::Mat & color_mat, cv::Mat & depth_mat, Registration mode, bool filter_points);
	void disableLog();
	void enableLog();

private:

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = nullptr;
	libfreenect2::PacketPipeline * pipeline_ = nullptr;
	libfreenect2::Registration * registration_ = nullptr;
	libfreenect2::Logger * logger_ = nullptr;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_, tmp_big_mat_;
	libfreenect2::SyncMultiFrameListener listener_;
	std::string serial_;
	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
	int tmp_map_[512 * 424]; 
	float qnan_;
};

struct K2G_Parameters
{
public:
	K2G_Parameters(K2G & k2g, K2G::Registration mode);

	libfreenect2::Freenect2Device::IrCameraParams ir;
	libfreenect2::Freenect2Device::ColorCameraParams rgb;
	cv::Size2i rgb_size;
	cv::Size2i ir_size;
	 
	cv::Mat getKir();
	cv::Mat getKrgb();
	cv::Mat getDir();
	cv::Mat getDrgb();
};

#endif