#include "all.h"

// To stop all the threads if one receives a stop signal
bool to_stop = false;
// Connection status
bool online = true;
// Enable visualization
bool visualization = false;
// Mongoose websocket port
const char * MONGOOSE_PORT= "8081";
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
	if(p.get("help")){
		printHelp();
		return 0;
	}
	
	visualization = p.get("visualization");

	if(argc == 1)
	{
		std::cout << "please specify at least one sensor source\n" << std::endl; 
		return -1;
	}

	// Check if the input template list file is correct
	std::ifstream infile;
	if(!p.getString("object").empty())
		infile.open(p.getString("object"));

	if(!infile && p.get("object"))
	{
		std::cout << "cannot open template list file: " << p.getString("object") << std::endl;
		return -1;
	}

	// Keep aliver
	std::thread ws_writer(asiothreadfx);

	// Start time
	start = std::chrono::system_clock::now();

	// Kinect Frame acquisition
	KinectManagerExchange * kinect_manager;
	if(p.get("object"))
	{
		kinect_manager = new KinectManagerExchange();
		kinect_manager->start();
	}

	// Check the endpoint string and connect to the collector
	std::string end_point = "http://pelars.sssup.it:8080/pelars/";
	end_point = end_point.back() == '/' ? end_point : end_point + "/";
	std::cout << "WebServer endpoint : " << end_point << std::endl;

	//Creating a Session Manager and getting a newsession ID
	SessionManager sm(end_point);
	int session = sm.getNewSession();

	// Create QR code
	if(p.get("qr") && session != -1){
		//drawQr(session);
		drawQr_(512, 8, session);
	}
	else if(session == -1)
		std::cout << "No Qr code available since there is no active internet connection " << std::endl;

	std::cout << "Mongoose websocket started on port 8081" << std::endl;
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
	nc = mg_bind(&mgr, "8081", ev_handler);
	mg_set_protocol_http_websocket(nc);
	std::cout << "Mongoose websocket on port 8081" << std::endl;

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
		thread_list.push_back(std::thread(handDetector, std::ref(collector)));
	// Starting the ide logger
	if(p.get("ide"))
		thread_list.push_back(std::thread(ideHandler, std::ref(mgr)));
	// Starting audio detector
	if(p.get("audio"))
		thread_list.push_back(std::thread(audioDetector, std::ref(collector)));
	
	//If there are no windows wait for Esc to be pressed
	if(!visualization && !p.get("special")){
		std::string str = "";
		char ch;
		while((ch = std::cin.get()) != 27)
		{ 
			if (to_stop)
				break;
		}
		to_stop = true;
	}
 
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
	std::cout << "IO stopped" << std::endl; 

	return 0;
}



