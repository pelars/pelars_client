#pragma once
#include <opencv2/core/core.hpp>        
#include <string>

struct ImageFrame{

	cv::Mat color;
	cv::Mat depth;
	std::string type;
};