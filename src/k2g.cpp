#include "k2g.h"

libfreenect2::Freenect2Device::ColorCameraParams kinect2parameters;


#ifdef HAS_FREENECT2
K2G::K2G(Processor p): undistorted_(512, 424, 4), registered_(512, 424, 4), big_mat_(1920, 1082, 4), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
qnan_(std::numeric_limits<float>::quiet_NaN())
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


	prepareMake3D(dev_->getIrCameraParams());

	registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

	kinect2parameters = getRgbParameters();

}


void K2G::shutDown()
{
	dev_->stop();
	dev_->close();
}

cv::Mat K2G::getColor()
{
	listener_.waitForNewFrame(frames_);
	libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
	cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
	cv::Mat r = tmp.clone();
	listener_.release(frames_);
	return r;
}

// Depth and color are aligned and registered 
void K2G::get(cv::Mat & color_mat, cv::Mat & depth_mat, const bool full_hd, const bool remove_points){
	listener_.waitForNewFrame(frames_);
	libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
	libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

	registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

	cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
	cv::Mat tmp_color;
	if(full_hd)
		tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
	else
		tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

	cv::flip(tmp_depth, depth_mat, 1);
	color_mat = tmp_color.clone();
	
	listener_.release(frames_);
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

void K2G::disableLog() {
		logger_ = libfreenect2::getGlobalLogger();
		libfreenect2::setGlobalLogger(nullptr);
	}

void K2G::enableLog() {
	libfreenect2::setGlobalLogger(logger_);
}

void K2G::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
{
	const int w = 512;
	const int h = 424;
    float * pm1 = colmap.data();
    float * pm2 = rowmap.data();
    for(int i = 0; i < w; i++)
    {
        *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
    }
    for (int i = 0; i < h; i++)
    {
        *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
    }
}

#endif






