#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <boost/network/protocol/http/client.hpp>
#include "kinect_grabber.h"
#include "data_writer.h"
#include <iostream>
#include "omp_parallel_for.h"
#include <json/json.h>
#include "enc_dec.hpp"

extern bool to_stop;
extern bool visualization;
extern double interval;
extern std::chrono::time_point<std::chrono::system_clock> start;

// Draw the Linemod results
void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T, int objid);

// Functions to store detector and templates in single XML/YAML file
cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename);

// Linemod result container
struct result_t
{
	cv::linemod::Match m;
	std::chrono::time_point<std::chrono::system_clock> end;
};

int linemodf(std::ifstream & infile, KinectManagerExchange * kme, DataWriter & websocket);