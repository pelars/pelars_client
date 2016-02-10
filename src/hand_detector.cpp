#include "hand_detector.h"

void handDetector(DataWriter & websocket, float marker_size, ImageSender & image_sender, K2G::Processor processor)
{
	aruco::MarkerDetector MDetector;
	MDetector.setMinMaxSize(0.01, 0.7);
	vector<aruco::Marker> markers;
	if(visualization)
		cv::namedWindow("hands");

	const int session = websocket.getSession();
	
	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "hand";

	float tx, ty, tz;
	std::string message;

	K2G k2g(processor);

	TimedSender timer(interval / 2);
	TimedSender timer_minute(60000);


	std::string folder_name = std::string("../../images/snapshots_") + std::to_string(session);

	cv::Mat camera_parameters = cv::Mat::eye(3, 3, CV_32F);
	camera_parameters.at<float>(0,0) = k2g.getRgbParameters().fx; 
	camera_parameters.at<float>(1,1) = k2g.getRgbParameters().fy; 
	camera_parameters.at<float>(0,2) = k2g.getRgbParameters().cx; 
	camera_parameters.at<float>(1,2) = k2g.getRgbParameters().cy;
	cv::Mat grey, color;
	bool to_send;

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
	
	cv::Mat camera_inverse = calib_matrix.inv();
	cv::Mat marker_pose = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	while(!to_stop)
	{
		color = k2g.getColor();
		cvtColor(color, grey, CV_BGR2GRAY);
		cv::flip(color, color, 1);
		cv::flip(grey, grey, 1);	
		if((snapshot_table && image_sender) || timer_minute.needSend()){
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			std::string name = std::string(folder_name + "/workspace_" + now + "_" + std::to_string(session) + ".jpg");
			if(!boost::filesystem::exists(folder_name)){
				boost::filesystem::path dir(folder_name);
				boost::filesystem::create_directory(dir);
			}
			imwrite(name, color);
			if(online){
				std::ifstream in(name, std::ifstream::binary);
				in.unsetf(std::ios::skipws);
			    in.seekg(0, std::ios::end);
			    std::streampos fileSize = in.tellg();
			    in.seekg(0, std::ios::beg);
			    std::vector<char> data(fileSize);
				in.read(&data[0], fileSize);
				std::string code = base64_encode((unsigned char*)&data[0], (unsigned int)data.size());
				image_sender.send(code, "jpg", "workspace");
			}
			snapshot_table = false;
		}

		MDetector.detect(grey, markers, camera_parameters, cv::Mat(), marker_size);

		if(markers.size() > 0){
			to_send = timer.needSend();
			for(unsigned int i = 0; i < markers.size(); ++i)
			{
				// Get marker position
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
				
				//std::cout << x << " " << y << " " << z << " " << markers[i].id << std::endl;
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
						io.post([&websocket, message]() {
							websocket.writeData(message);
							});
						}
					websocket.writeLocal(message);
				}
			}
		}

		if(visualization){
			cv::imshow("hands", color);
			int c = cv::waitKey(1);
			if((char)c == 'q' )
			{
				to_stop = true;
				std::cout << "Stop requested by hand detector" << std::endl;
			}
		}
	}

	//Destroy the window
	cvDestroyWindow("hands");
	k2g.shutDown();
	return;
}

		