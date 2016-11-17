#include "opt.h"

// Take a camera snapshot of the table
bool snapshot_table = false;
// Take a camera snapshot of the people
bool snapshot_people = false;
// Take a snapshot of the screen
bool snapshot_screen = false;
// Connection status
bool online = true;
// Enable visualization
bool visualization = false;
// System starting time
const std::string currentDateTimeNow = currentDateTime();
// sending interval
double interval = 1000;
// Asion communication service and Asio keep alive
boost::asio::io_service io;

// Keep alive
void aliver(const boost::system::error_code& /*e*/)
{
	
}

void asiothreadfx()
{
	boost::asio::deadline_timer t(io, boost::posix_time::seconds(100000));
	t.async_wait(aliver);
	io.run();
}

void sessionWriter(int session){

	while(!to_stop){
		std::cout << "session is " << session << std::endl;
		sleep(3);
	}
}

std::string time2string(std::chrono::high_resolution_clock::time_point tp){
	return std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count());
}

const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
	return buf;
}

// Check if escape was pressed to terminate execution
void checkEscape(bool visualization, bool special){
	if(!visualization && !special){
		while(std::cin.get() != 27){
			if(to_stop){
				break;
			}
		}
		terminateMe();
		std::cout << "Stopping" << std::endl;
	}
}

int sendCalibration(DataWriter & websocket, bool no_webcam, bool no_kinect2){

	cv::Mat wcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);
	cv::Mat kcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	cv::FileStorage wfile("../../data/calibration_webcam.xml", cv::FileStorage::READ);
	cv::FileStorage kfile("../../data/calibration_kinect2.xml", cv::FileStorage::READ);

	Json::StyledWriter writer;

	if(!no_webcam){
		if(wfile.isOpened())
		{	
			wfile["matrix"] >> wcalib_matrix;
			wfile.release();
		}else{
			std::cout << "could not find face calibration file; use -c to calibrate the cameras" << std::endl;
			return -1;
		}
	}
	if(!no_kinect2){
		if(kfile.isOpened())
		{	
			kfile["matrix"] >> kcalib_matrix;
			kfile.release();
		}else{
			std::cout << "could not find hand calibration file; use -c to calibrate the cameras" << std::endl;
			return -1;
		}
	}

	Json::Value root;
	Json::Value array = Json::arrayValue;

	root["obj"]["type"] = "calibration";
	root["obj"]["camera"] = "webcam";
	for(size_t i = 0; i < 3; ++i)
		for(size_t j = 0; j < 4; ++j)
			array.append(wcalib_matrix.at<float>(i,j));

	root["obj"]["parameters"] = array; 

	std::string message = writer.write(root);
	//std::cout << "sending " << message << std::endl;	
	
	// Send the message online and store it offline
	if(online){
		//std::cout << "Hand detector posting data to the server\n " << std::flush;
		io.post( [&websocket, message]() {
			websocket.writeData(message);
		});
	}
	websocket.writeLocal(message);
	
	Json::Value array2 = Json::arrayValue;
	root["obj"]["camera"] = "kinect2";
	for(size_t i = 0; i < 3; ++i)
		for(size_t j = 0; j < 4; ++j)
			array2.append(kcalib_matrix.at<float>(i,j));

	root["obj"]["parameters"] = array2; 
	message = writer.write(root);
	//std::cout << "sending " << message << std::endl;

	// Send the message online and store it offline
	if(online){
		//std::cout << "Hand detector posting data to the server\n " << std::flush;
		io.post( [&websocket, message]() {
			websocket.writeData(message);
			});
		}
	websocket.writeLocal(message);

	return 0;
}


void drawStatus(Parser & p){

	sleep(2);
	cv::startWindowThread();
	cv::namedWindow("status");

	int offset = 30;
	cv::Mat status(240, 320 , CV_8UC1);
    status.setTo(cv::Scalar(255, 255, 255));
    if(p.get("face") || p.get("default")){
    	cv::putText(status, "-face tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("audio") || p.get("default")){
    	cv::putText(status, "-audio tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("hand") || p.get("default")){
    	cv::putText(status, "-hand tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("particle")){
    	cv::putText(status, "-particle tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
 		offset += 30;
 	}
    if(p.get("qr") || p.get("default")){
    	cv::putText(status, "-qr visualization", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("session")){
    	cv::putText(status, "-continuing previous session", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("calibration")){
    	cv::putText(status, "-calibration", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("ide") || p.get("default")){
    	cv::putText(status, "-arduino tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }

    cv::imshow("status", status);
	while(!to_stop){
		int c = cv::waitKey(1);
		if((char)c == 'q') {
			terminateMe();
			cv::destroyWindow("status");
		}
	}
}