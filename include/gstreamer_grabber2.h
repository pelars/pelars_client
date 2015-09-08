#pragma once
#include <gst/gst.h> 
#include <gst/app/gstappsink.h>
#include <glib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>        
#include <vector>
#include <string>

class GstreamerGrabber2{
	
public:


	GstreamerGrabber2(const char * device, const int width, const int height, const bool yuv420, const bool testsrc, const char * command = "");
	~GstreamerGrabber2();

	void capture(IplImage * frame);
	void operator >>(cv::Mat & frame);

private:

	GstElement *pipeline_ = 0;
	GstElement * src_ = 0;
	GstElement * sink_ = 0;
	GstAppSink * appsink_ = 0;
	GstMessage * msg_ = 0;
	GstBus * bus_ = 0;
	GError * error_ = 0;
	std::vector<uint8_t> data_;
	char buffer_[1024];
	int width_;
	int height_;
	bool yuv420_;
	bool testsrc_;
};