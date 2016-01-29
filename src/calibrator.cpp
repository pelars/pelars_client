#include "calibrator.h"

void calibration(const unsigned int id, const float marker_size){

	std::cout << "press c to calibrate when the marker is seen in both cameras" << std::endl;
	
	K2G k2g(K2G::OPENCL);

	cv::FileStorage kinect_file("../../data/calibration_kinect2.xml", cv::FileStorage::WRITE);
	cv::FileStorage webcam_file("../../data/calibration_webcam.xml", cv::FileStorage::WRITE);
	cv::Mat kgray, kcolor;

	const int width = 800;
	const int height = 448;

	GstreamerGrabber gs_grabber(width, height, id);


	aruco::MarkerDetector MDetector;
	MDetector.setMinMaxSize(0.01, 0.7);
	vector<aruco::Marker> kmarkers;
	vector<aruco::Marker> wmarkers;

	// Kinect2 parameters
	cv::Mat kcamera_parameters = cv::Mat::eye(3, 3, CV_32F);
	kcamera_parameters.at<float>(0,0) = k2g.getRgbParameters().fx; 
	kcamera_parameters.at<float>(1,1) = k2g.getRgbParameters().fy; 
	kcamera_parameters.at<float>(0,2) = k2g.getRgbParameters().cx; 
	kcamera_parameters.at<float>(1,2) = k2g.getRgbParameters().cy;

	const float fx = 589.3588305153235;
	const float cx = 414.1871817694326;
	const float fy = 588.585116717914;
	const float cy = 230.3588624031242; 

	cv::Mat wcamera_parameters = cv::Mat::eye(3, 3, CV_32F);
	wcamera_parameters.at<float>(0,0) = fx; 
	wcamera_parameters.at<float>(1,1) = fy; 
	wcamera_parameters.at<float>(0,2) = cx; 
	wcamera_parameters.at<float>(1,2) = cy;


	cv::Mat kcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);
	cv::Mat wcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);


	bool kfound = false;
	bool wfound = false;
	bool stop = false;

	cv::namedWindow("kinect2");
	cv::namedWindow("webcam");

	while(!stop){
		// Kinect2 grabber
		kcolor = k2g.getColor();
		cvtColor(kcolor, kgray, CV_BGR2GRAY);
		MDetector.detect(kgray, kmarkers, kcamera_parameters, cv::Mat(), marker_size);

		// Webcam grabber
		IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
		gs_grabber.capture(frame);
		cv::Mat wgray(frame); 
		MDetector.detect(wgray, wmarkers, wcamera_parameters, cv::Mat(), marker_size);

		// TODO
		if(kmarkers.size() > 0){
			for(unsigned int i = 0; i < kmarkers.size(); ++i){
				if(kmarkers[i].id == 0){
					kcalib_matrix.at<float>(0, 3) = kmarkers[i].Tvec.at<float>(0);
					kcalib_matrix.at<float>(1, 3) = kmarkers[i].Tvec.at<float>(1);
					kcalib_matrix.at<float>(2, 3) = kmarkers[i].Tvec.at<float>(2);
					cv::Rodrigues(kmarkers[i].Rvec, cv::Mat(kcalib_matrix, cv::Rect(0, 0, 3, 3)));		
					kfound = true;
					kmarkers[i].draw(kgray, cv::Scalar(0, 0, 255), 2);
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
					wmarkers[i].draw(wgray, cv::Scalar(0, 0, 255), 2);
					std::cout << wcalib_matrix << std::endl;
					break;
				}
			}		
		}

		cv::imshow("kinect2", kgray);
		cv::imshow("webcam", wgray);
		int c = cv::waitKey(1);
		if((char)c == 'c' && wfound && kfound) {
			kinect_file << "matrix" << kcalib_matrix;
			kinect_file.release();
			webcam_file << "matrix" << wcalib_matrix;
			webcam_file.release();
			stop = true;	
			std::cout << "CAMERAS CALIBRATED SUCCESSFULLY" << std::endl;
		}else{
			wfound = false;
			kfound = false;
		}
	}

	cvDestroyWindow("kinect2");
	cvDestroyWindow("webcam");
	k2g.shutDown();

}