#include "k2g.h"

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
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr K2G::getCloud()
{		
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_);
		const unsigned short w = undistorted_.width;
		const unsigned short h = undistorted_.height;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

		const float * itD0 = (float *)undistorted_.data;
		const char * itRGB0 = (char *)registered_.data;
		pcl::PointXYZRGB * itP = &cloud->points[0];
		bool is_dense = true;
		
		for(int y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;
				
				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
	
					const float rx = colmap(x) * depth_value;
                	const float ry = dy * depth_value;               
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				} else {
					itP->z = qnan_;
					itP->x = qnan_;
					itP->y = qnan_;

					itP->b = qnan_;
					itP->g = qnan_;
					itP->r = qnan_;
					is_dense = false;
 				}
			}
		}
		cloud->is_dense = is_dense;
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






