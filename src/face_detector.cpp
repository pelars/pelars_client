#include "face_detector.h"

double std_width = 185.0; //mm

double focal_length_pixel = 589.3588305153235; //489.3;  //pixel

// Returns the distance in meters
inline double distance(int x1, int x2){
		return ((std_width * focal_length_pixel) / std::abs(x1 - x2)) / 1000;
	}

void detectFaces(DataWriter & websocket, ScreenGrabber & screen_grabber, ImageSender & image_sender_screen, 
	             ImageSender & image_sender_people, const int face_camera_id, const bool video)
{

	cv::Mat calib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	cv::FileStorage file("../../data/calibration_webcam.xml", cv::FileStorage::READ);
	if(file.isOpened())
	{	
		file["matrix"] >> calib_matrix;
		file.release();
	}else{
		std::cout << "could not find face calibration file; use -c to calibrate the cameras" << std::endl;
		to_stop = true;
	}

	// Needed since else opencv does not crete the window (BUG?)
	if(visualization)
		sleep(1);

	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	//std::string face_cascade_name_ = "../../data/haarcascade_frontalface_alt.xml";
	std::string face_cascade_name_gpu_ = "../../data/haarcascade_frontalface_alt2.xml";
	cv::CascadeClassifier face_cascade_;

	cv::gpu::GpuMat gray_gpu;
	cv::Mat faces_downloaded, color;

	const int session = websocket.getSession();

	const bool findLargestObject_ = false;
	const bool filterRects_ = true;

	cv::gpu::CascadeClassifier_GPU cascade_gpu_;
	cascade_gpu_.visualizeInPlace = false;
	cascade_gpu_.findLargestObject = findLargestObject_;

	const unsigned int width = 800;
	const unsigned int height = 448;

	const float fx = 589.3588305153235;
	const float cx = 414.1871817694326;
	const float fy = 588.585116717914;
	const float cy = 230.3588624031242; 
/*
	cv::VideoWriter * outputVideo;
	if(video){
		int fourcc = CV_FOURCC('M','J','P','G');
		int fps = 30;
		std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
		std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
		outputVideo->open("webcam_"+ now + "_" + std::to_string(session) + ".avi", fourcc, fps, cv::Size(width,height));
	}
*/
	std::string image_folder_name = std::string("../../images");
	std::string image_subfolder_name = std::string("../../images/snapshots_") + std::to_string(session);

	std::string video_folder_name = std::string("../../videos");
	std::string video_subfolder_name = std::string("../../videos/videos_") + std::to_string(session); 
	

	std::shared_ptr<x264Encoder> x264encoder;
	if(video){

		if(!boost::filesystem::exists(video_folder_name)){
			boost::filesystem::path dir(video_folder_name);
			boost::filesystem::create_directory(dir);
		}

		if(!boost::filesystem::exists(video_subfolder_name)){
			boost::filesystem::path dir(video_subfolder_name);
			boost::filesystem::create_directory(dir);
		}
		std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
		std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
		x264encoder = std::make_shared<x264Encoder>(video_subfolder_name + "/", "webcam"+ now + "_" + std::to_string(session) + ".h264");
		x264encoder->initialize(width, height, false);
	}

	/*
	const float k1 = 0.12269303;
	const float k2 = -0.26618881;
	const float p1 = 0.00129035;
	const float p2 = 0.00081791;
	const float k3 = 0.17005303;
	*/

	Json::Value upper;
	Json::Value root = Json::arrayValue;
	Json::Value array;

	float face_distance, tx, ty, tz, tx1, ty1, tz1, tx2, ty2, tz2, x_unproject, y_unproject;
	cv::Mat pose;
	cv::Mat face_pose = cv::Mat(cv::Size(1, 4), CV_32F);
	face_pose.at<float>(0, 3) = 1;
	int detections_num;

	// Preapare JSON message to send to the Collectorh
	std::string code;

	Json::StyledWriter writer;

	GstreamerGrabber gs_grabber(width, height, face_camera_id);
	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3); 

	if(!cascade_gpu_.load(face_cascade_name_gpu_))
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_gpu_ << std::endl; 
		to_stop = true;
	}
	
	if(visualization)
		cv::namedWindow("face");

	TimedSender timer(interval);
	TimedSender timer_minute(60000);

	cv::Mat camera_inverse = calib_matrix.inv();
	
	while(!to_stop)
	{	

		cv::gpu::GpuMat facesBuf_gpu;
		gs_grabber.capture(frame);
		color = cv::Mat(frame);

		//cv::flip(color, color, 1);
/*
		if(video)
			outputVideo->write(color);
*/	
		if(video){
			x264encoder->encodeFrame((const char *)color.data, 3);
		}

		cv::gpu::GpuMat color_gpu(color);

		//cvtColor(color, gray, CV_BGR2GRAY);
		cv::gpu::cvtColor(color_gpu, gray_gpu, CV_BGR2GRAY);

		bool send_minute = timer_minute.needSend();
		if((snapshot_people && image_sender_people) || send_minute){
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			
			if(!boost::filesystem::exists(image_folder_name)){
				boost::filesystem::path dir(image_folder_name);
				boost::filesystem::create_directory(dir);
			}

			if(!boost::filesystem::exists(image_subfolder_name)){
				boost::filesystem::path dir(image_subfolder_name);
				boost::filesystem::create_directory(dir);
			}

			std::string name = std::string(image_subfolder_name + "/people_" + now + "_" + std::to_string(session) +".jpg");
			cv::imwrite(name, color);
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
					image_sender_people.send(code, "jpg", "people", true);
				else
					image_sender_people.send(code, "jpg", "people", false);
			}
			snapshot_people = false;
		}
		if((snapshot_screen && image_sender_screen) || send_minute){
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			std::string name = std::string(image_subfolder_name + "/screen_" + now + "_" + std::to_string(session) + ".png");
			
			if(!boost::filesystem::exists(image_folder_name)){
				boost::filesystem::path dir(image_folder_name);
				boost::filesystem::create_directory(dir);
			}

			if(!boost::filesystem::exists(image_subfolder_name)){
				boost::filesystem::path dir(image_subfolder_name);
				boost::filesystem::create_directory(dir);
			}
			
			screen_grabber.grabScreen(name);
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
					image_sender_screen.send(code, "png", "screen", true);
				else
					image_sender_screen.send(code, "png", "screen", false);
			}
			snapshot_screen = false;
		}
		send_minute = false;
		
		detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, cv::Size(color.cols,color.rows), cv::Size(), 1.05, (filterRects_ || findLargestObject_) ? 4 : 0);

		facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
		cv::Rect * faces = faces_downloaded.ptr<cv::Rect>();

		
		//std::cout << " found " << detections_num << " faces" << std::endl;
		// Take same time for all faces
		std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
		array["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();
		for(int i = 0; i < detections_num; ++i)
		{

			cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
			cv::ellipse(color, center, cv::Size(faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, cv::Scalar( 255, 0, 255), 4, 8, 0);

			// Distance in meters
			face_distance = distance(faces[i].x, faces[i].x + faces[i].width);

			const float const_x = (face_distance / fx);
			const float const_y = (face_distance / fy);

			x_unproject = (faces[i].x - cx) * const_x;
			y_unproject = (faces[i].y - cy) * const_y;

			//std::cout << x_unproject << " " << y_unproject << std::endl;
				
			const float tmp = y_unproject;
			
			//face_pose.at<float>(0, 3) = 1; already done after declaration
			face_pose.at<float>(0, 0) = x_unproject;
			face_pose.at<float>(0, 1) = y_unproject;
			face_pose.at<float>(0, 2) = face_distance;

			pose = camera_inverse * face_pose;
			tx = pose.at<float>(0, 0);
			ty = pose.at<float>(0, 1);
			tz = pose.at<float>(0, 2);

			x_unproject = (faces[i].x + faces[i].width - cx) * const_x;
			y_unproject = (faces[i].y + faces[i].height - cy) * const_y;

			face_pose.at<float>(0, 0) = x_unproject;
			face_pose.at<float>(0, 1) = y_unproject;

			pose = camera_inverse * face_pose;
			tx1 = pose.at<float>(0, 0);
			ty1 = pose.at<float>(0, 1);
			tz1 = pose.at<float>(0, 2);

			// Reuse previous values
			face_pose.at<float>(0, 0) = x_unproject;
			face_pose.at<float>(0, 1) = tmp;

			pose = camera_inverse * face_pose;
			tx2 = pose.at<float>(0, 0);
			ty2 = pose.at<float>(0, 1);
			tz2 = pose.at<float>(0, 2);

			// Json message
			array["type"] = "face";
			array["id"] = i;
			array["x"] = tx; 
			array["y"] = -ty;
			array["z"] = tz;
			array["x1"] = tx1;
			array["y1"] = -ty1;
			array["z1"] = tz1;
			array["x2"] = tx2;
			array["y2"] = -ty2;
			array["z2"] = tz2;
			array["distance"] = face_distance;
			
			cv::circle(color, cv::Point(faces[i].x, faces[i].y), 10, cv::Scalar(0,0,255), -1);
			cv::circle(color, cv::Point(faces[i].x + faces[i].height, faces[i].y + faces[i].width), 10, cv::Scalar(0,255,0), -1);
			cv::circle(color, cv::Point(faces[i].x + faces[i].height, faces[i].y ), 10, cv::Scalar(255,0,0), -1);
			cv::circle(color, cv::Point(faces[i].x, faces[i].y + faces[i].width ), 10, cv::Scalar(255,255,0), -1);
/*
			std::cout << "RED x " << tx << " ,y " << ty << ", tz " << tz << std::endl;
			std::cout << "GREEN x1 " << tx1 << " ,y1 " << ty1 << ", tz " << tz1 << std::endl;
			std::cout << "BLUE x2 " << tx2 << " ,y2 " << ty2 << ", tz " << tz2 << std::endl;
			std::cout << face_distance << std::endl;
			*/
			root.append(array);
		}
		
		upper["obj"] = root;
		// Send message
		if(detections_num && timer.needSend()){
			std::string out_string = writer.write(upper);
			if(online){
				io.post( [&websocket, out_string]() {
				websocket.writeData(out_string);
				});
			}
			websocket.writeLocal(out_string);  
		}

		//gray_gpu.release();
		//facesBuf_gpu.release();
		upper.clear();
		root.clear();
		array.clear();

		// Show what you got
		if(visualization){
			cv::imshow("face", color);
			int c = cv::waitKey(30);
			if((char)c == 'q' ) {
				to_stop = true;
				std::cout << "stop requested by face detector" << std::endl;
			}
		}
	}

	if(video)
		x264encoder->unInitilize();
	if(visualization)
		cvDestroyWindow("face");

	return;
}

