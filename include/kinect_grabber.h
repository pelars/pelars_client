#pragma once
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <opencv2/imgproc/imgproc.hpp> 
#include "threadpool.h"
#include "opt.h"

class KinectManagerExchange
{
public:

	KinectManagerExchange();
	void stop();
	void start();
	bool get(cv::Mat & color, cv::Mat & depth);
	bool getColorRGB(cv::Mat & color);
	bool getColorBGR(cv::Mat & color);
	bool getColorGRAY(cv::Mat & color);

private:
	
	int fd_;
	int size_;
	std::thread manager_;
	bool stop_ = false;
	unsigned char * common_ = 0;
	boost::interprocess::shared_memory_object shm_obj_;
	boost::interprocess::mapped_region region_;
	bool subready_ = false;
	bool processed_ = false;
	bool imageready_ = false;
	std::mutex m_;
	std::condition_variable cv_;
};
