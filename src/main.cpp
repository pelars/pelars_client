#include "all.h"

// To stop all the threads if one receives a stop signal
bool to_stop = false;
// Connection status
bool online = true;
// Enable visualization
bool visualization = false;
// System starting time
const std::string currentDateTimeNow = currentDateTime();
// sending interval
double interval = 1000;
// initial time point
std::chrono::time_point<std::chrono::system_clock> start;
// signal handler
static void sig_handler(int signum)
{
		to_stop = true;
		printf("Received signal %d\n", signum);
}

int main(int argc, char * argv[])
{
	signal(SIGHUP, sig_handler);
	signal(SIGTERM, sig_handler);

	Parser p(argc, argv);
	if(p.get("help") || argc == 1){
		printHelp();
		return 0;
	}

	float marker_size = p.get("marker") ? p.getFloat("marker") : 0.040;	
	visualization = p.get("visualization");

	// Check if the input template list file is correct
	std::ifstream infile;
	if(p.get("object")){
		infile.open(p.getString("object"));
		if(!infile)
		{
			std::cout << "cannot open template list file: " << p.getString("object") << std::endl;
			printHelp();
			return -1;
		}
	}

	// Start time
	start = std::chrono::system_clock::now();

	// Kinect Frame acquisition
	KinectManagerExchange * kinect_manager;
	if(p.get("object"))
	{
		kinect_manager = new KinectManagerExchange();
		if(*kinect_manager)
			kinect_manager->start();
		else{
			return -1;
		}
	}

	// Keep aliver
	std::thread ws_writer(asiothreadfx);

	// Check the endpoint string and connect to the collector
	std::string end_point = p.get("Server") ? p.getString("Server") : "http://pelars.sssup.it:8080/pelars/";
	end_point = end_point.back() == '/' ? end_point : end_point + "/";
	std::cout << "WebServer endpoint : " << end_point << std::endl;

	//Creating a Session Manager and getting a newsession ID
	SessionManager sm(end_point);
	int session = sm.getNewSession();

	// Create QR code
	if(p.get("qr") && session != -1)
		drawQr(512, 8, session);
	else if(session == -1 && p.get("qr"))
		std::cout << "No Qr code available since there is no active internet connection " << std::endl;

	std::cout << "Collector endpoint : " << end_point + "collector/" + to_string(session) << std::endl;

	// Check the endpoint string and connect to the session manager
	std::string session_endpoint = end_point + "session/";
	std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;     

	// Websocket manager
	DataWriter collector(end_point + "collector", session);

	// Mongoose websocket listener and message structure
	struct mg_mgr mgr;
	struct mg_connection * nc;
	MiniEncapsule writer(collector, session);
	mg_mgr_init(&mgr, &writer);
	const char * port = p.get("mongoose") ? p.getString("mongoose").c_str() : "8081";
	nc = mg_bind(&mgr, port, ev_handler);
	mg_set_protocol_http_websocket(nc);
	std::cout << "Mongoose websocket started on port " << std::string(port) << std::endl;

	// Thread container
	std::vector<std::thread> thread_list;

	// Starting the linemod thread
	if(p.get("object"))
		thread_list.push_back(std::thread(linemodf, std::ref(infile), kinect_manager, std::ref(collector)));
	// Starting the face detection thread
	if(p.get("face"))
		thread_list.push_back(std::thread(detectFaces, std::ref(collector)));
	// Starting the particle.io thread
	if(p.get("particle"))
		thread_list.push_back(std::thread(sseHandler, std::ref(collector)));
	// Starting the hand detector
	if(p.get("hand"))
		thread_list.push_back(std::thread(handDetector, std::ref(collector), marker_size, p.get("calibration")));
	// Starting the ide logger
	if(p.get("ide"))
		thread_list.push_back(std::thread(ideHandler, std::ref(mgr)));
	// Starting audio detector
	if(p.get("audio"))
		thread_list.push_back(std::thread(audioDetector, std::ref(collector)));
	
	//If there are no windows wait for Esc to be pressed
	checkEscape(visualization, p.get("special"));

	// Wait for the termination of all threads
	for(auto &thread : thread_list)
		thread.join();
	
	// Create a local file for data acquisition and backup
	std::string tmp = collector.file_name_ + std::string(online ? "_backup_" : "_local_") + currentDateTimeNow + collector.file_extention_;
	std::rename(collector.complete_file_name_.c_str(), tmp.c_str());

	// Terminate everything and exit
	// Close session
	sm.closeSession(session);
	// Stopping the kinect grabber
	if(p.get("object"))
		kinect_manager->stop();
	// Stopping the websocket
	collector.stop();
	std::cout << "Connection to Collector closed" << std::endl;
	// Stopped io service 
	io.stop();
	// Stopping Asio aliver
	ws_writer.join();
	return 0;
}



