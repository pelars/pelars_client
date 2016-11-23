#include "marker_viewer.h"
#include "opt.h"
#include <iostream>
#ifdef HAS_ARUCO
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#endif
#include "gstreamer_grabber.h"
#include <opencv2/imgproc/imgproc.hpp>

void show_markers(const unsigned int id, const float marker_size){
	
#ifdef HAS_ARUCO
	const int width = 1920;
	const int height = 1080;

	GstreamerGrabber gs_grabber(width, height, id);

	aruco::MarkerDetector MDetector;
	MDetector.setMinMaxSize(0.01, 0.7);
	vector<aruco::Marker> kmarkers;
	vector<aruco::Marker> wmarkers;

	//const float fx = 589.3588305153235;
	//const float cx = 414.1871817694326;
	//const float fy = 588.585116717914;
	//const float cy = 230.3588624031242;

	const float fx = 2102.85441;
	const float cx = 949.50000;
	const float fy = 2178.28254;
	const float cy = 724.50000;  

	cv::Mat wcamera_parameters = cv::Mat::eye(3, 3, CV_32F);
	wcamera_parameters.at<float>(0,0) = fx; 
	wcamera_parameters.at<float>(1,1) = fy; 
	wcamera_parameters.at<float>(0,2) = cx; 
	wcamera_parameters.at<float>(1,2) = cy;

	cv::namedWindow("webcam");

	cv::Mat wdist = cv::Mat(cv::Size(4, 1), CV_32F);
	wdist.at<float>(0) = 0.1161538110871388; 
	wdist.at<float>(1) = -0.213821121281364; 
	wdist.at<float>(2) = 0.000927392238536357; 
	wdist.at<float>(3) = 0.0007135216206840332;

	aruco::CameraParameters wparam(wcamera_parameters, wdist, cv::Size(width, height));

	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	bool stop = false;
	cv::Mat wgray;

	while(!stop){
		// Webcam grabber
		gs_grabber.capture(frame);
		cv::Mat wcolor(frame);
		cvtColor(wcolor, wgray, CV_BGR2GRAY); 
		MDetector.detect(wgray, wmarkers, wcamera_parameters, cv::Mat(), marker_size);

		if(wmarkers.size() > 0){
			for(unsigned int i = 0; i < wmarkers.size(); ++i){
				wmarkers[i].draw(wcolor, cv::Scalar(0, 0, 255), 2);
			}		
		}

		cv::imshow("webcam", wcolor);
		int c = cv::waitKey(1);
		if((char)c == 'q') {
			stop = true;	
		}
	}

	cvDestroyWindow("webcam");
	#endif
}