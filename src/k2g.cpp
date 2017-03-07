#include "k2g.h"

libfreenect2::Freenect2Device::ColorCameraParams kinect2parameters;


#ifdef HAS_FREENECT2
K2G::K2G(Processor p): undistorted_(512, 424, 4), registered_(512, 424, 4), tmp_big_mat_(1920, 1082, 4), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Depth),
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
void K2G::get(cv::Mat & color_mat, cv::Mat & depth_mat, Registration mode, bool filter_points)  {
	listener_.waitForNewFrame(frames_);
	libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
	libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

	switch(mode)
	{
		case Registration::None:
		{
			color_mat = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).clone();
			depth_mat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).clone();
			break;
		}
		case Registration::Undistorted:
		{
			registration_->undistorDepth(depth,&undistorted_);
			// return: undistorted_ + rgb
			//
			// rgb_ is owened by FRAME => need clone
			// undistorted_ is owned by K2G
			color_mat = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).clone();
			depth_mat = cv::Mat(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
			break;
		}
		case Registration::ColorToDepth:
		{
			registration_->apply(rgb, depth, &undistorted_, &registered_, filter_points, &tmp_big_mat_, tmp_map_);
			color_mat = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data); 
			depth_mat = cv::Mat(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
			// return. undistored_ + registred_ both in Depth Size
			//
			// both owned by K2G
			break;
		}
		case Registration::DepthToColor:
		{
			registration_->apply(rgb, depth, &undistorted_, &registered_, true, &tmp_big_mat_, tmp_map_);
			color_mat = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).clone();
			depth_mat = cv::Mat(rgb->height,tmp_big_mat_.width, CV_32FC1, tmp_big_mat_.data); // to have it 1080
			// return. tmp_big_mat_ + rgb both in Color Size
			// NOTE: need to remove last two lines of tmp_big_mat_
			// rgb_ is owened by FRAME => need clone
			// tmp_big_mat_ is owned by K2G
			break;
		}
	}
	listener_.release(frames_);
}

libfreenect2::Freenect2Device::IrCameraParams K2G::getIrParameters()
{
	return dev_->getIrCameraParams();
}

libfreenect2::Freenect2Device::ColorCameraParams K2G::getRgbParameters()
{
	return dev_->getColorCameraParams();
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

K2G_Parameters::K2G_Parameters(K2G & k2g, K2G::Registration mode)
{
	auto cp = k2g.getRgbCameraParams();
	auto dp = k2g.getIrParameters();
	switch(mode)
	{
		case K2G::RegistrationMode::None:
		{
			rgb =cp;
			ir = dp;
			rgb_size = {1920,1080};
			ir_size = {512,424};
			break;
		}
		case K2G::RegistrationMode::Undistorted:
		{
			rgb =cp;
			ir = dp;
			ir.k1 = 0;
			ir.k2 = 0;
			ir.k3 = 0;
			ir.p1 = 0;
			ir.p2 = 0;
			rgb_size = {1920,1080};
			ir_size = {512,424};
			break;
		}
		case K2G::RegistrationMode::ColorToDepth:
		{
			ir = dp;
			ir.k1 = 0;
			ir.k2 = 0;
			ir.k3 = 0;
			ir.p1 = 0;
			ir.p2 = 0;
			rgb = ir;
			ir_size = rgb_size = {512,424};
			break;
		}
		case K2G::RegistrationMode::DepthToColor:
		{
			rgb = cp;
			ir = rgb;
			ir_size = rgb_size = {1920,1080};
			break;
		}	
	}
}

cv::Mat K2G_Parameters::getKir()
{
	cv::Mat m = cv::Mat::eye(3, 3, CV_32F);
	m.at<float>(0,0) = ir.fx;
	m.at<float>(1,1) = ir.fy;
	m.at<float>(0,2) = ir.cx;
	m.at<float>(1,2) = ir.cy;
	return m;
}

cv::Mat K2G_Parameters::getKrgb()
{
	cv::Mat m = cv::Mat::eye(3, 3, CV_32F);
	m.at<float>(0,0) = rgb.fx;
	m.at<float>(1,1) = rgb.fy;
	m.at<float>(0,2) = rgb.cx;
	m.at<float>(1,2) = rgb.cy;
	return m;
}

cv::Mat K2G_Parameters::getDir()
{
	cv::Mat m = cv::Mat::eye(5, 1, CV_32F);
	m.at<float>(0,0) = rgb.k1;
	m.at<float>(1,0) = rgb.k2;
	m.at<float>(2,0) = rgb.k3;
	m.at<float>(3,0) = rgb.p1;
	m.at<float>(4,0) = rgb.p2;
	return m;
}

cv::Mat K2G_Parameters::getDrgb()
{
	cv::Mat m = cv::Mat::eye(5, 1, CV_32F);
	m.at<float>(0,0) = ir.k1;
	m.at<float>(1,0) = ir.k2;
	m.at<float>(2,0) = ir.k3;
	m.at<float>(3,0) = ir.p1;
	m.at<float>(4,0) = ir.p2;
	return m;
}


#endif






