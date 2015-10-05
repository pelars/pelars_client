#include "hand_detector.h"

void handDetector(DataWriter & websocket, float marker_size)
{
	aruco::MarkerDetector MDetector;
	vector<aruco::Marker> markers;
	if(visualization)
		cv::namedWindow("hands");
	
	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "hand";

	float x, y, z;
	std::string message;

	//GstreamerGrabber gs_grabber(1920, 1080, 1);
	//GstreamerGrabber2 gs_grabber2("/dev/video0", 1920, 1080, true, false);
	//IplImage * frame = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 1); //TODO 
	//Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("../../data/calibration/rgb_calibration.yaml", "../../data/calibration/depth_calibration.yaml", "../../data/calibration/pose_calibration.yaml");
	
	K2G k2g(OPENGL);

	TimedSender timer(interval);
	//z = 0.0f;

	cv::Mat camera_parameters = cv::Mat::eye(3, 3, CV_32F);
	camera_parameters.at<float>(0,0) = k2g.getRgbParameters().fx; 
	camera_parameters.at<float>(1,1) = k2g.getRgbParameters().fy; 
	camera_parameters.at<float>(0,2) = k2g.getRgbParameters().cx; 
	camera_parameters.at<float>(1,2) = k2g.getRgbParameters().cy;
	cv::Mat grey, color;
	bool to_send;
	while(!to_stop)
	{
		grey = k2g.getGrey();
		MDetector.detect(grey, markers, camera_parameters, cv::Mat(), marker_size);

		if(markers.size() > 0)
			to_send = timer.needSend();
		if(markers.size() > 0){
			for(int i = 0; i < markers.size(); ++i)
			{
				// Get marker position
				markers[i].draw(grey, cv::Scalar(0, 0, 255), 2);
				//aruco::CvDrawingUtils::draw3dCube(grey, markers[i], camera);

				x = markers[i].Tvec.ptr<float>(0)[0];
				y = markers[i].Tvec.ptr<float>(0)[1];
				z = markers[i].Tvec.ptr<float>(0)[2];
				std::cout << x << " " << y << " " << z << std::endl;
				if(to_send){
					root["obj"]["id"] = markers[i].id;
					root["obj"]["x"] = x;
					root["obj"]["y"] = y;
					root["obj"]["z"] = z / 1000;
					root["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
					message = writer.write(root);

					// Send the message online and store it offline
					if(online){
						//std::cout << "Hand detector posting data to the server\n " << std::flush;
						io.post( [&websocket, message]() {
							websocket.writeData(message);
							});
						}
					websocket.writeLocal(message);
				}
			}
		}

		if(visualization){
			cv::imshow("hands", grey);
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

		