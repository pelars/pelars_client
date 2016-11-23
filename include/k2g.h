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

	enum Processor{
		CPU, OPENCL, OPENGL
	};

	K2G(Processor p);
	libfreenect2::Freenect2Device::IrCameraParams getIrParameters();
	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
	void shutDown();
	cv::Mat getColor();
	void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);
	void get(cv::Mat & color_mat, cv::Mat & depth_mat, const bool full_hd = true, const bool remove_points = false);
	void disableLog();
	void enableLog();

private:

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = nullptr;
	libfreenect2::PacketPipeline * pipeline_ = nullptr;
	libfreenect2::Registration * registration_ = nullptr;
	libfreenect2::Logger * logger_ = nullptr;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_, big_mat_;
	libfreenect2::SyncMultiFrameListener listener_;
	std::string serial_;
	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
	int map_[512 * 424]; 
	float qnan_;

};

#endif