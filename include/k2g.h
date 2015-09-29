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

extern bool to_stop;
extern bool TURBO_COLOR;

enum processor{
	CPU, OPENCL, OPENGL
};

class K2G {

public:

	K2G(processor p);
	libfreenect2::Freenect2Device::IrCameraParams getIrParameters();
	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();
	void shutDown();
	cv::Mat getColor();
	cv::Mat getGrey();
private:

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::PacketPipeline * pipeline_ = 0;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::SyncMultiFrameListener listener_;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_;
	Eigen::Vector4d pworld_;
	Eigen::Matrix4d d_matrix_, d_matrix_inv_;
	std::string serial_;
};
