#include "opt.h"

// To stop all the threads if one receives a stop signal
bool to_stop = false;
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

// Signal handler
void sig_handler(int signum)
{
		to_stop = true;
		printf("Received signal %d\n", signum);
}

void asiothreadfx()
{
	boost::asio::deadline_timer t(io, boost::posix_time::seconds(100000));
	t.async_wait(aliver);
	io.run();
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
		to_stop = true;
		std::cout << "Stopping" << std::endl;
	}
}

int sendCalibration(DataWriter & websocket){

	cv::Mat wcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);
	cv::Mat kcalib_matrix = cv::Mat::eye(cv::Size(4, 4), CV_32F);

	cv::FileStorage wfile("../../data/calibration_webcam.xml", cv::FileStorage::READ);
	cv::FileStorage kfile("../../data/calibration_kinect2.xml", cv::FileStorage::READ);

	Json::StyledWriter writer;

	if(wfile.isOpened())
	{	
		wfile["matrix"] >> wcalib_matrix;
		wfile.release();
	}else{
		std::cout << "could not find face calibration file; use -c to calibrate the cameras" << std::endl;
		return -1;
	}

	if(kfile.isOpened())
	{	
		kfile["matrix"] >> kcalib_matrix;
		kfile.release();
	}else{
		std::cout << "could not find hand calibration file; use -c to calibrate the cameras" << std::endl;
		return -1;
	}

	Json::Value root;
	Json::Value array = Json::arrayValue;

	root["obj"]["type"] = "calibration";
	root["obj"]["camera"] = "webcam";
	for(unsigned int i = 0; i < 3; ++i)
		for(unsigned int j = 0; j < 4; ++j)
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
	for(unsigned int i = 0; i < 3; ++i)
		for(unsigned int j = 0; j < 4; ++j)
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
