#include "kinect_grabber.h"

// Asion communication service and Asio keep alive
boost::asio::io_service io;

KinectManagerExchange::KinectManagerExchange(): shm_obj_(boost::interprocess::open_or_create, "region", boost::interprocess::read_write)
{
	size_ = 640 * 480 * 6 + 4;
	shm_obj_.truncate(size_);
	boost::interprocess::mapped_region r(shm_obj_, boost::interprocess::read_write);
	r.swap(region_);
	common_ = (unsigned char *)region_.get_address();
	std::cout << "Shared pointer for kinect data is " << region_.get_address() << std::endl;
}

void KinectManagerExchange::stop() 
{
	std::unique_lock<std::mutex> lk(m_);
	stop_ = true;
	processed_ = true;
	cv_.notify_one();
}

void KinectManagerExchange::start()
{
  //std::thread t(&KinectManagerExchange::subhandler,this);
  //manager.swap(t);
}

bool KinectManagerExchange::getColorRGB(cv::Mat & color)
{
	static int lastframe = -2;
	while(true)
	{
		int frame = *(int*)common_;
		if(frame == lastframe)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		lastframe = frame;
		break;
	}

	color.create(480, 640, CV_8UC3);                
	int k1 = 3 * 640 * 480;
	memcpy(color.data, common_ + 4, k1);
	cv::cvtColor(color, color, CV_BGR2RGB); //this will put colors right

	return true;
}

bool KinectManagerExchange::getColorBGR(cv::Mat & color)
{
	static int lastframe = -2;
	while(true)
	{
		int frame = *(int*)common_;
		if(frame == lastframe)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		lastframe = frame;
		break;
	}

	color.create(480, 640, CV_8UC3);                
	int k1 = 3 * 640 * 480;
	memcpy(color.data, common_ + 4, k1);

	return true;
}

bool KinectManagerExchange::getColorGRAY(cv::Mat & color)
{
	static int lastframe = -2;
	while(true)
	{
		int frame = *(int*)common_;
		if(frame == lastframe)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		lastframe = frame;
		break;
	}

	color.create(480, 640, CV_8UC3);                
	int k1 = 3 * 640 * 480;
	memcpy(color.data, common_ + 4, k1);
	cv::cvtColor(color, color, CV_BGR2GRAY); //this will put colors right

	return true;
}

bool KinectManagerExchange::get(cv::Mat & color, cv::Mat & depth)
{
	static int lastframe = -2;
	while(true)
	{
		int frame = *(int*)common_;
		if(frame == lastframe)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			continue;
		}
		lastframe = frame;
		break;
	}

	color.create(480, 640, CV_8UC3);                
	depth.create(480, 640, CV_16U);
	int k1 = 3 * 640 * 480;
	memcpy(color.data, common_ + 4, k1);
	cv::cvtColor(color, color, CV_BGR2RGB); //this will put colors right
	int k2 = 2 * 640 * 480;
	memcpy(depth.data, common_+ k1 + 4, k2);

	return true;
}

