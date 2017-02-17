#pragma once
#include <opencv2/core/core.hpp>        
#include <string>
#include <chrono>

struct CamParams{

	CamParams(cv::Mat cam_matrix, cv::Mat dist, unsigned int width, unsigned int height):
				cam_matrix_(cam_matrix), dist_(dist), width_(width), height_(height) {}

	CamParams(){}

	cv::Mat cam_matrix_, dist_;
	unsigned int width_;
	unsigned int height_;
};

struct ImageFrame{

	bool hasColor(){
		return (color_.cols * color_.rows) > 0;
	}
	bool hasDepth(){
		return (depth_.cols * depth_.rows) > 0;
	}

	cv::Mat color_, depth_;
	std::string type_;
	CamParams params_;
	long seq_number_;

	std::chrono::high_resolution_clock::time_point time_stamp_;
};


