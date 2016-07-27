#include "calibrator.h"

void calibration(const unsigned int face_camera_id, const unsigned int hand_camera_id, const float marker_size, bool c920, K2G::Processor processor){

	const int width = 800;
	const int height = 448;

	std::cout << "press c to calibrate when the marker is seen in both cameras" << std::endl;
	GstreamerGrabber gs_grabber(width, height, face_camera_id);
	sleep(5);

	std::shared_ptr<K2G> k2g;
	std::shared_ptr<GstreamerGrabber> gs_grabber_hand;

	// Check if c920 is used for hands
	if(c920){
		gs_grabber_hand = std::make_shared<GstreamerGrabber>(1920, 1080, hand_camera_id);
	}else{
		k2g = std::make_shared<K2G>(processor);
	}

	cv::Mat kgray, kcolor, wgray;

	aruco::MarkerDetector MDetector;
	MDetector.setMinMaxSize(0.01, 0.7);
	std::vector<aruco::Marker> kmarkers;
	std::vector<aruco::Marker> wmarkers;


	// Kinect2 parameters
	cv::Mat kcamera_parameters = cv::Mat::eye(3, 3, CV_32F);

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

	cv::namedWindow("hands");
	cv::namedWindow("faces");

	cv::Mat kdist = cv::Mat(cv::Size(4, 1), CV_32F);
	cv::Mat wdist = cv::Mat(cv::Size(4, 1), CV_32F);

	kdist.at<float>(0) = 0; 
	kdist.at<float>(1) = 0; 
	kdist.at<float>(2) = 0; 
	kdist.at<float>(3) = 0;

	wdist.at<float>(0) = 0.1161538110871388; 
	wdist.at<float>(1) = -0.213821121281364; 
	wdist.at<float>(2) = 0.000927392238536357; 
	wdist.at<float>(3) = 0.0007135216206840332;

	shared_ptr<aruco::CameraParameters> kparam;

	if(c920){
		kparam = std::make_shared<aruco::CameraParameters>(wcamera_parameters, kdist, cv::Size(1920,1080));
	}else{
		kcamera_parameters.at<float>(0,0) = k2g->getRgbParameters().fx; 
		kcamera_parameters.at<float>(1,1) = k2g->getRgbParameters().fy; 
		kcamera_parameters.at<float>(0,2) = k2g->getRgbParameters().cx; 
		kcamera_parameters.at<float>(1,2) = k2g->getRgbParameters().cy;
		kparam = std::make_shared<aruco::CameraParameters>(kcamera_parameters, kdist, cv::Size(1920,1080));
	}

	
	aruco::CameraParameters wparam(wcamera_parameters, wdist, cv::Size(width, height));

	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	IplImage * frame2 = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 3);


	while(!stop){
		// Kinect2 grabber
		if(c920){
			gs_grabber_hand->capture(frame2);
			kcolor = cv::Mat(frame2);
			cv::flip(kcolor, kcolor, 1);
		}else{
			kcolor = k2g->getColor();
		}

		cvtColor(kcolor, kgray, CV_BGR2GRAY);

		MDetector.detect(kgray, kmarkers, kcamera_parameters, cv::Mat(), marker_size);

		// Webcam grabber
		gs_grabber.capture(frame);
		cv::Mat wcolor(frame);
		cv::flip(wcolor, wcolor, 1);
		cvtColor(wcolor, wgray, CV_BGR2GRAY); 
		MDetector.detect(wgray, wmarkers, wcamera_parameters, cv::Mat(), marker_size);

		// TODO
		if(kmarkers.size() > 0){
			for(unsigned int i = 0; i < kmarkers.size(); ++i){
				if(kmarkers[i].id == 0){
					kcalib_matrix.at<float>(0, 3) = kmarkers[i].Tvec.at<float>(0);
					kcalib_matrix.at<float>(1, 3) = kmarkers[i].Tvec.at<float>(1);
					kcalib_matrix.at<float>(2, 3) = kmarkers[i].Tvec.at<float>(2);
					std::cout << kmarkers[i].Tvec.at<float>(0) << " " << kmarkers[i].Tvec.at<float>(1) << " " 
					          << kmarkers[i].Tvec.at<float>(2) << std::endl;
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
					//std::cout << wmarkers[i].Tvec.at<float>(0) << " " << wmarkers[i].Tvec.at<float>(1) << " " << wmarkers[i].Tvec.at<float>(2) << std::endl;
					cv::Rodrigues(wmarkers[i].Rvec, cv::Mat(wcalib_matrix, cv::Rect(0, 0, 3, 3)));					
					wfound = true;
					wmarkers[i].draw(wcolor, cv::Scalar(0, 0, 255), 2);
					aruco::CvDrawingUtils::draw3dAxis(wcolor, wmarkers[i], wparam);
					break;
				}
			}		
		}

		cv::imshow("hands", kcolor);
		cv::imshow("faces", wcolor);
		int c = cv::waitKey(10);
		if((char)c == 'c' && wfound && kfound) {
			cv::FileStorage kinect_file("../../data/calibration_kinect2.xml", cv::FileStorage::WRITE);
			cv::FileStorage webcam_file("../../data/calibration_webcam.xml", cv::FileStorage::WRITE);
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

	cvDestroyWindow("hands");
	cvDestroyWindow("faces");
	if(k2g)
		k2g->shutDown();
}