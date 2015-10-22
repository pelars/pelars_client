#include "k2g.h"

K2G::K2G(processor p): undistorted_(512, 424, 4), registered_(512, 424, 4), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth)
{
	if(freenect2_.enumerateDevices() == 0)
	{
		std::cout << "no kinect2 connected!" << std::endl;
		exit(-1);
	}

	serial_ = freenect2_.getDefaultDeviceSerialNumber();
	switch(p){
		case CPU:
			std::cout << "creating CPU processor" << std::endl;
			pipeline_ = new libfreenect2::CpuPacketPipeline();
			break;
		case OPENCL:
			std::cout << "creating OpenCL processor" << std::endl;
			pipeline_ = new libfreenect2::OpenCLPacketPipeline();
			break;
		case OPENGL:
			std::cout << "creating OpenGL processor" << std::endl;
			pipeline_ = new libfreenect2::OpenGLPacketPipeline();
			break;
		default:
			std::cout << "creating CPU processor" << std::endl;
			pipeline_ = new libfreenect2::CpuPacketPipeline();
			break;
	}
	
	dev_ = freenect2_.openDevice(serial_, pipeline_);
	dev_->setColorFrameListener(&listener_);
	dev_->setIrAndDepthFrameListener(&listener_);
	dev_->start();

	registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

	d_matrix_ = Eigen::Matrix4d::Identity();
	d_matrix_(0,0) = dev_->getIrCameraParams().fx;
	d_matrix_(0,2) = dev_->getIrCameraParams().cx;
	d_matrix_(1,1) = dev_->getIrCameraParams().fy;
	d_matrix_(1,2) = dev_->getIrCameraParams().cy;
	d_matrix_inv_ = d_matrix_.inverse();
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr K2G::getCloud()
{		
	listener_.waitForNewFrame(frames_);
	libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
	libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

	registration_->apply(rgb, depth, &undistorted_, &registered_);
	const short w = undistorted_.width;
	const short h = undistorted_.height;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

	const float * itD0 = (float *)undistorted_.data;
	const char * itRGB0 = (char *)registered_.data;
	pcl::PointXYZRGB * itP = &cloud->points[0];
	
	for(int y = 0; y < h; ++y)
	{	
		const unsigned int offset = y * w;
		const float * itD = itD0 + offset;
		const char * itRGB = itRGB0 + offset*4;
		
		for(size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
		{
			const float depth_value = *itD / 1000.0f;
			
			if(isnan(depth_value))
			{
				itP->x = itP->y = itP->z = std::numeric_limits<float>::quiet_NaN();
				itP->rgba = 0;

			}else{

				Eigen::Vector4d psd(x, y, 1.0, 1.0 / depth_value);
				pworld_ = d_matrix_inv_ * psd * depth_value;
				itP->z = depth_value;

				itP->x = isnan(pworld_.x()) ? 0 : pworld_.x();
				itP->y = isnan(pworld_.y()) ? 0 : pworld_.y();

				itP->b = itRGB[0];
				itP->g = itRGB[1];
				itP->r = itRGB[2];
			}
		}
	}

	listener_.release(frames_);
	return cloud;
}

void K2G::shutDown()
{
	dev_->stop();
	dev_->close();
}

cv::Mat K2G::getColor()
{
	//TURBO_COLOR = true;
	listener_.waitForNewFrame(frames_);
	libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
	cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
	cv::Mat r = tmp.clone();
	listener_.release(frames_);
	return std::move(r);
}

cv::Mat K2G::getGrey()
{
	//TURBO_COLOR = false;
	listener_.waitForNewFrame(frames_);
	libfreenect2::Frame * grey = frames_[libfreenect2::Frame::Color];
	cv::Mat tmp(grey->height, grey->width, CV_8UC1, grey->data);
	cv::Mat r = tmp.clone();
	listener_.release(frames_);
	return std::move(r);
}

libfreenect2::Freenect2Device::IrCameraParams K2G::getIrParameters()
{
	libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
	return ir;
}

libfreenect2::Freenect2Device::ColorCameraParams K2G::getRgbParameters()
{
	libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
	return rgb;
}