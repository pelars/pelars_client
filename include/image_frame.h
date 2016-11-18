#pragma once
#include <opencv2/core/core.hpp>        
#include <string>
#include <chrono>

struct ImageFrame{

	bool hasColor(){
		return (color_.cols * color_.rows) > 0;
	}
	bool hasDepth(){
		return (depth_.cols * depth_.rows) > 0;
	}

	cv::Mat color_, depth_;
	std::string type_;

	std::chrono::high_resolution_clock::time_point time_stamp_;
};


