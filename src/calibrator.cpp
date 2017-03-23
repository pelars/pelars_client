#include "calibrator.h"
#include <iostream>
#if defined(HAS_ARUCO) && defined(HAS_FREENECT2)
#include "k2g.h"
#include "opt.h"
#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <boost/filesystem.hpp>
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>  
#include "gstreamer_grabber.h"



void calibration(const unsigned int face_camera_id, const unsigned int hand_camera_id, const float marker_size, bool c920, 
	             K2G::Processor processor, bool force)
{
	std::cout << "press c to calibrate when the marker is seen in both cameras" << std::endl;
	
	const int width = 1920;
	const int height = 1080;

	GstreamerGrabber gs_grabber(width, height, face_camera_id);

	std::shared_ptr<K2G> k2g;
	std::shared_ptr<GstreamerGrabber> gs_grabber_hand;

	cv::Mat kgray, kcolor, wgray;

	aruco::MarkerDetector MDetector;
	MDetector.setMinMaxSize(0.01, 0.7);
	std::vector<aruco::Marker> kmarkers, wmarkers;

	// Kinect2 parameters
	cv::Mat kcamera_parameters = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat wcamera_parameters = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat kdist = cv::Mat(cv::Size(4, 1), CV_32F);
	cv::Mat wdist = cv::Mat(cv::Size(4, 1), CV_32F);

	kdist.at<float>(0) = 0.0; 
	kdist.at<float>(1) = 0.0; 
	kdist.at<float>(2) = 0.0; 
	kdist.at<float>(3) = 0.0;

	if(!boost::filesystem::exists("../../data/c920_parameters.xml")){
		std::cout << "ERROR: ../../data/c920_parameters.xml does not exist, using default ones" << std::endl;
		const float fx = 1352.73;
		const float cx = 985.184;
		const float fy = 1352.73;
		const float cy = 544.005;

		wcamera_parameters.at<float>(0,0) = fx; 
		wcamera_parameters.at<float>(1,1) = fy; 
		wcamera_parameters.at<float>(0,2) = cx; 
		wcamera_parameters.at<float>(1,2) = cy;

		wdist.at<float>(0) = 0.1161538110871388; 
		wdist.at<float>(1) = -0.213821121281364; 
		wdist.at<float>(2) = 0.000927392238536357; 
		wdist.at<float>(3) = 0.0007135216206840332;

	} else{
		std::cout << "Found valid c920 calibration file" << std::endl;
		cv::FileStorage in("../../data/c920_parameters.xml", cv::FileStorage::READ);
		in["cameraMatrix1920x1080"] >> wcamera_parameters;
		in["distCoeff1920x1080"] >> wdist;

		std::cout << "Loaded face camera parameters : " << std::endl;
		std::cout << wcamera_parameters << std::endl;

		std::cout << "Loaded face distortion parameters : " << std::endl;
		std::cout << wdist << std::endl;
	}

	cv::Mat kcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);
	cv::Mat wcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	bool kfound = false;
	bool wfound = false;
	bool stop = false;

	cv::namedWindow("hands");
	cv::namedWindow("faces");

	shared_ptr<aruco::CameraParameters> kparam;

	if(c920){
		gs_grabber_hand = std::make_shared<GstreamerGrabber>(1920, 1080, hand_camera_id);
		kparam = std::make_shared<aruco::CameraParameters>(wcamera_parameters, wdist, cv::Size(1920,1080));
	}else{
		k2g = std::make_shared<K2G>(processor);
		kcamera_parameters.at<float>(0,0) = k2g->getRgbParameters().fx; 
		kcamera_parameters.at<float>(1,1) = k2g->getRgbParameters().fy; 
		kcamera_parameters.at<float>(0,2) = k2g->getRgbParameters().cx; 
		kcamera_parameters.at<float>(1,2) = k2g->getRgbParameters().cy;
		kparam = std::make_shared<aruco::CameraParameters>(kcamera_parameters, kdist, cv::Size(1920,1080));
	}

	std::cout << "Loaded hand camera parameters : " << std::endl;
	std::cout << kcamera_parameters << std::endl;

	std::cout << "Loaded hand distortion parameters : " << std::endl;
	std::cout << kdist << std::endl;
	
	aruco::CameraParameters wparam(wcamera_parameters, wdist, cv::Size(width, height));

	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage * frame2 = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 3);
	int counter = 0;

	while(!stop){
		// Kinect2 grabber
		if(c920){
			gs_grabber_hand->capture(frame2);
			kcolor = cv::Mat(frame2);
		}else{
			kcolor = k2g->getColor();
			cv::flip(kcolor, kcolor, 1);
		}

		cvtColor(kcolor, kgray, CV_BGR2GRAY);

		MDetector.detect(kgray, kmarkers, kcamera_parameters, cv::Mat(), marker_size);

		// Webcam grabber
		gs_grabber.capture(frame);
		cv::Mat wcolor(frame);
		cvtColor(wcolor, wgray, CV_BGR2GRAY); 
		MDetector.detect(wgray, wmarkers, wcamera_parameters, cv::Mat(), marker_size);

		if(kmarkers.size() > 0){
			for(unsigned int i = 0; i < kmarkers.size(); ++i){
				if(kmarkers[i].id == 0){
					kcalib_matrix.at<float>(0, 3) = kmarkers[i].Tvec.at<float>(0);
					kcalib_matrix.at<float>(1, 3) = kmarkers[i].Tvec.at<float>(1);
					kcalib_matrix.at<float>(2, 3) = kmarkers[i].Tvec.at<float>(2);

					cv::Rodrigues(kmarkers[i].Rvec, cv::Mat(kcalib_matrix, cv::Rect(0, 0, 3, 3)));		
					kfound = true;
					kmarkers[i].draw(kcolor, cv::Scalar(0, 0, 255), 2);
					aruco::CvDrawingUtils::draw3dAxis(kcolor, kmarkers[i], *kparam);
					break;
				}
			}		
		}
		if(wmarkers.size() > 0){
			for(unsigned int i = 0; i < wmarkers.size(); ++i){
				if(wmarkers[i].id == 0){
					wcalib_matrix.at<float>(0, 3) = wmarkers[i].Tvec.at<float>(0);
					wcalib_matrix.at<float>(1, 3) = wmarkers[i].Tvec.at<float>(1);
					wcalib_matrix.at<float>(2, 3) = wmarkers[i].Tvec.at<float>(2);
					cv::Rodrigues(wmarkers[i].Rvec, cv::Mat(wcalib_matrix, cv::Rect(0, 0, 3, 3)));					
					wfound = true;
					wmarkers[i].draw(wcolor, cv::Scalar(0, 0, 255), 2);
					aruco::CvDrawingUtils::draw3dAxis(wcolor, wmarkers[i], wparam);
					break;
				}
			}		
		}

		if(!force){
			cv::imshow("hands", kcolor);
			cv::imshow("faces", wcolor);
			int c = cv::waitKey(10);
			if((char)c == 'c' && wfound && kfound) {
				store(kcalib_matrix, wcalib_matrix);
				stop = true;
			}else{
				wfound = false;
				kfound = false;
			}

			if((char)c == 'q') {
				std::cout << "ABORTING!!!!" << std::endl;
				exit(-1);
			}

		}else{
			//Used when there is no way of vieweing the two images and we just want a stable calibration (10 stable frames)
			if(wfound && kfound){
				counter++;
			}else{
				wfound = false;
				kfound = false;
				counter = 0;
			}

			if(counter > 10) {
				store(kcalib_matrix, wcalib_matrix);
				stop = true;
			}
		}

	}

	cvDestroyWindow("hands");
	cvDestroyWindow("faces");
	if(k2g)
		k2g->shutDown();
}
#endif

void store(const cv::Mat & kcalib_matrix, const cv::Mat & wcalib_matrix){

	cv::FileStorage kinect_file("../../data/calibration_kinect2.xml", cv::FileStorage::WRITE);
	cv::FileStorage webcam_file("../../data/calibration_webcam.xml", cv::FileStorage::WRITE);
	kinect_file << "matrix" << kcalib_matrix;
	kinect_file.release();
	webcam_file << "matrix" << wcalib_matrix;
	webcam_file.release();	
	std::cout << "CAMERAS CALIBRATED SUCCESSFULLY" << std::endl;
}