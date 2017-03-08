
#include "hand_detector.h"
#include "timer_manager.h"

#ifdef HAS_ARUCO
void handDetector(DataWriter & websocket, float marker_size, 
				  std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc, const bool c920, unsigned int camera_id)
{	

	const unsigned int width = 1920;
	const unsigned int height = 1080;

	synchronizer.lock();
	cv::Mat calib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	cv::FileStorage file("../../data/calibration_kinect2.xml", cv::FileStorage::READ);
	if(file.isOpened())
	{	
		file["matrix"] >> calib_matrix;
		file.release();
	}else{
		std::cout << "could not find hand calibration file; use -c to calibrate the cameras" << std::endl;
		to_stop = true;
	}

	aruco::MarkerDetector marker_detector;
	marker_detector.setMinMaxSize(0.01, 0.1);
	marker_detector.setDesiredSpeed(3);

	bool inited = false;

	vector<aruco::Marker> markers;
	if(visualization)
		cv::namedWindow("hands");

	const int session = websocket.getSession();
	
	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "hand";

	float tx, ty, tz;
	std::string message;

	std::shared_ptr<GstreamerGrabber> gs_grabber;
	IplImage * frame;

	TimedSender timer(interval / 2);

	std::string video_folder_name = std::string("../../videos");
	std::string video_subfolder_name = std::string("../../videos/videos_") + std::to_string(session); 

	// Fixed size since it has to match the kinect2 resolution
	if(c920){
		gs_grabber = std::make_shared<GstreamerGrabber>(width, height, camera_id);
	}
	
	frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3); 
	cv::Mat camera_parameters;

	cv::Mat grey, color;
	bool to_send;

	cv::Mat camera_inverse = calib_matrix.inv();
	cv::Mat marker_pose = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	std::shared_ptr<ImageFrame> frames;

	synchronizer.unlock();


	TimerManager * tm = TimerManager::instance();

	while(!to_stop)
	{
		if(c920){
			gs_grabber->capture(frame);
			color = cv::Mat(frame);
		} else {
			// Could return since it is terminated
			pc->read(frames);
			color = frames->color_;
		}
		TimerScope ts(tm,"handDetector");

		if(!inited){
			camera_parameters = frames->color_params_.cam_matrix_;
			inited = true;
		}

		 
		
		cvtColor(color, grey, CV_BGR2GRAY);

		marker_detector.detect(grey, markers, camera_parameters, cv::Mat(), marker_size);

		if(markers.size() > 0){

			to_send = timer.needSend();
			for(unsigned int i = 0; i < markers.size(); ++i)
			{
				if(markers[i].id != 0)
				{
					// Get marker position
					if(visualization)
						markers[i].draw(color, cv::Scalar(0, 0, 255), 2);
					//aruco::CvDrawingUtils::draw3dCube(grey, markers[i], camera);
					
					marker_pose.at<float>(0, 3) = markers[i].Tvec.at<float>(0);
					marker_pose.at<float>(1, 3) = markers[i].Tvec.at<float>(1);
					marker_pose.at<float>(2, 3) = markers[i].Tvec.at<float>(2);
					cv::Rodrigues(markers[i].Rvec, cv::Mat(marker_pose, cv::Rect(0, 0, 3, 3)));

					cv::Mat pose = camera_inverse * marker_pose;
					tx = pose.at<float>(0, 3);
					ty = pose.at<float>(1, 3);
					tz = pose.at<float>(2, 3);
					Eigen::Matrix3f temp;
					cv2eigen(pose(cv::Rect(0,0,3,3)), temp);
					Eigen::Quaternionf quaternion(temp);
					
					//std::cout << tx << " " << ty << " " << tz << std::endl;	

					if(to_send){
						root["obj"]["id"] = markers[i].id;
						root["obj"]["tx"] = tx;
						root["obj"]["ty"] = ty;
						root["obj"]["tz"] = tz;
						root["obj"]["rw"] = quaternion.w();
						root["obj"]["rx"] = quaternion.x();
						root["obj"]["ry"] = quaternion.y();
						root["obj"]["rz"] = quaternion.z();
						std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
						root["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
						message = writer.write(root);
						
						// Send the message online and store it offline
						if(online){
							//std::cout << "Hand detector posting data to the server\n " << std::flush;
							//std::cout << "hand detector sending " << message << std::endl;
							io.post([&websocket, message]() {
								websocket.writeData(message);
								});
							}
						websocket.writeLocal(message);
					}
				}
			}
			
		}

		if(visualization){
			cv::imshow("hands", color);
			int c = cv::waitKey(30);
			if((char)c == 'q' )
			{
				terminateMe();
				std::cout << "Stop requested by hand detector" << std::endl;
			}
		}
	}

	//Destroy the window
	if(visualization)
		cvDestroyWindow("hands");
}

#endif
		