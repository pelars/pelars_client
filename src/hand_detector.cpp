#include "hand_detector.h"

void handDetector(DataWriter & websocket, float marker_size, ImageSender & image_sender, K2G::Processor processor, const bool video, 
	              const bool c920, unsigned int camera_id)
{

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

	vector<aruco::Marker> markers;
	if(visualization)
		cv::namedWindow("hands");

	const int session = websocket.getSession();
	
	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "hand";

	float tx, ty, tz;
	std::string message;

	K2G * k2g;
	GstreamerGrabber * gs_grabber;
	IplImage * frame;

	if(c920){
		gs_grabber = new GstreamerGrabber(1920, 1080, camera_id);
		frame = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 3); 
	}
	else
	{
		k2g = new K2G(processor);
	}

	TimedSender timer(interval / 2);
	TimedSender timer_minute(60000);

	std::string folder_name = std::string("../../images/snapshots_") + std::to_string(session);
	std::string video_folder_name = std::string("../../videos");
	std::string video_subfolder_name = std::string("../../videos/videos_") + std::to_string(session); 
 
	x264Encoder * x264encoder;
	if(video){

		if(!boost::filesystem::exists(video_folder_name)){
			boost::filesystem::path dir(video_folder_name);
			boost::filesystem::create_directory(video_folder_name);
		}

		if(!boost::filesystem::exists(video_subfolder_name)){
			boost::filesystem::path dir(video_subfolder_name);
			boost::filesystem::create_directory(video_subfolder_name);
		}
		std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
		std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
		x264encoder = new x264Encoder(video_subfolder_name + "/", "kinect2_"+ now + "_" + std::to_string(session) + ".h264");
		x264encoder->initialize(1920, 1080, c920 ? false : true);
	}


	cv::Mat camera_parameters = cv::Mat::eye(3, 3, CV_32F);
	camera_parameters.at<float>(0,0) = c920 ? 589.3588305153235 : k2g->getRgbParameters().fx; 
	camera_parameters.at<float>(1,1) = c920 ? 588.5851167179140 : k2g->getRgbParameters().fy; 
	camera_parameters.at<float>(0,2) = c920 ? 414.1871817694326 : k2g->getRgbParameters().cx; 
	camera_parameters.at<float>(1,2) = c920 ? 230.3588624031242 : k2g->getRgbParameters().cy;
	cv::Mat grey, color;
	bool to_send;

	
	cv::Mat camera_inverse = calib_matrix.inv();
	cv::Mat marker_pose = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	while(!to_stop)
	{
		if(c920){
			gs_grabber->capture(frame);
			color = cv::Mat(frame);
		} else {
			color = k2g->getColor();		
		}

		cv::flip(color, color, 1); 
		
		if(video){
			if(c920)
				x264encoder->encodeFrame((const char *)color.data, 3);
			else
				x264encoder->encodeFrame((const char *)color.data, 4);
		}

		cvtColor(color, grey, CV_BGR2GRAY);
		bool send_minute = timer_minute.needSend();
		if((snapshot_table && image_sender) || send_minute){
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
				if(send_minute)
					image_sender.send(code, "jpg", "workspace", true);
				else
					image_sender.send(code, "jpg", "workspace", false);
			}
			snapshot_table = false;
		}

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
	if(k2g)
		k2g->shutDown();
	if(video)
		x264encoder->unInitilize();
	return;
}

		