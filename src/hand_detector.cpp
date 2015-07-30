#include "hand_detector.h"

void handDetector(KinectManagerExchange & kme, DataWriter & websocket, cv::VideoCapture & capture, int session){

	// OpenCV matrixes to contain the data acquired from the kinect
	cv::Mat color, depth;

	aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    sleep(1);
    cv::namedWindow("hands");

	while(!to_stop){
		/*
		if(!kme.get(color, depth)){
	      std::cout << "failed to fetch data from the kinect\n";
	      to_stop = true;
	      break;
	    }*/
	    capture >> color;
	    //std::cout << color.cols << "x" << color.rows << std::endl;
	    //MDetector.detect(color, markers, cameraMatrix_, distCoeffs_, 0.035, true);
	    
	    MDetector.detect(color, markers);
	    if(markers.size() > 0){
   			for(auto &m : markers){
				m.draw(color, cv::Scalar(0,0,255),2);
				
				cv::waitKey(1);//wait for key to be pressed

   			}
			
			
   			/*
			cv::Rodrigues(markers[0].Rvec, markers[0].Rvec);
			model_view_[0] = markers[0].Rvec.at<float>(0,0);
			model_view_[1] = markers[0].Rvec.at<float>(0,1);
			model_view_[2] = markers[0].Rvec.at<float>(0,2);
			model_view_[4] = markers[0].Rvec.at<float>(1,0);
			model_view_[5] = markers[0].Rvec.at<float>(1,1);
			model_view_[6] = markers[0].Rvec.at<float>(1,2);
			model_view_[8] = markers[0].Rvec.at<float>(2,0);
			model_view_[9] = markers[0].Rvec.at<float>(2,1);
			model_view_[10] = markers[0].Rvec.at<float>(2,2);

			model_view_[12] = 0;
			model_view_[13] = 0;
			model_view_[14] = 0;
			model_view_[15] = 1;*/
			
    	}
    	cv::imshow("hands", color);
	}
}

    