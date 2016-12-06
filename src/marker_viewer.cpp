#include "marker_viewer.h"
#include "opt.h"
#include <iostream>
#ifdef HAS_ARUCO
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#endif
#include "gstreamer_grabber.h"
#include <opencv2/imgproc/imgproc.hpp>

void show_markers(const unsigned int id, const float marker_size, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pcw){
	
#ifdef HAS_ARUCO

	aruco::MarkerDetector MDetector;
	MDetector.setMinMaxSize(0.01, 0.7);
	vector<aruco::Marker> kmarkers;
	vector<aruco::Marker> wmarkers;

	cv::Mat camera_parameters, dist;
	camera_parameters = cv::Mat::eye(3, 3, CV_32F);

	cv::namedWindow("aruco ids");
	
	bool stop = false;
	bool inited = false;
	cv::Mat wgray, wcolor;

	std::shared_ptr<ImageFrame> color_frame;

	while(!stop){

		if(!pcw->read(color_frame))
			continue;

		if(!inited){
			auto params = color_frame->params_;
			camera_parameters = params.cam_matrix_;
			dist = params.dist_;
			inited = true;
		}

		wcolor = color_frame->color_.clone();

		cvtColor(wcolor, wgray, CV_BGR2GRAY); 
		MDetector.detect(wgray, wmarkers, camera_parameters, dist, marker_size);

		if(wmarkers.size() > 0){
			for(unsigned int i = 0; i < wmarkers.size(); ++i){
				wmarkers[i].draw(wcolor, cv::Scalar(0, 0, 255), 2);
			}		
		}

		cv::imshow("aruco ids", wcolor);
		int c = cv::waitKey(30);
		if((char)c == 'q') {
			stop = true;	
		}
	}

	cvDestroyWindow("aruco ids");
	#endif
}