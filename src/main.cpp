#include "all.h"

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
// Signal handler
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
		p.printHelp();
		return 0;
	}

	// Check the endpoint string and connect to the collector
	std::string end_point = p.get("Server") ? p.getString("Server") : "http://pelars.sssup.it:8080/pelars/";
	end_point = end_point.back() == '/' ? end_point : end_point + "/";
	std::cout << "WebServer endpoint : " << end_point << std::endl;


	if(p.get("upload")){
		return uploadData(p.getString("upload"), end_point);
	}

	visualization = p.get("visualization");

	// Check if the input template list file is correct
	std::ifstream infile;
	if(p.get("object")){
		infile.open(p.getString("object"));
		if(!infile)
		{
			std::cout << "cannot open template list file: " << p.getString("object") << std::endl;
			p.printHelp();
			return -1;
		}
	}


	// Creating a Session Manager and getting a newsession ID
	SessionManager sm(end_point);
	sm.login();
	int session;
	if(!p.get("session"))
		session = sm.getNewSession();
	else{
		session = p.getInt("session");
		std::cout << "Using session " << session << std::endl;
	}
	std::string token = sm.getToken(); 

	// Image grabber
	ImageSender image_sender_table(session, end_point, token);
	ImageSender image_sender_people(session, end_point, token);
	ImageSender image_sender_screen(session, end_point, token);

	// Screen grabber
	ScreenGrabber screen_grabber;

	// Keep aliver
	std::thread ws_writer(asiothreadfx);

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

	std::cout << "Collector endpoint : " << end_point + "collector/" + to_string(session) << std::endl;
	// Check the endpoint string and connect to the session manager
	std::string session_endpoint = end_point + "session/";
	std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;     

	// Websocket manager
	DataWriter collector(end_point + "collector", session);

	// Thread container
	std::vector<std::thread> thread_list;

	// Starting the linemod thread
	if(p.get("object"))
		thread_list.push_back(std::thread(linemodf, std::ref(infile), kinect_manager, std::ref(collector)));
	// Starting the face detection thread
	if(p.get("face"))
		thread_list.push_back(std::thread(detectFaces, std::ref(collector), std::ref(screen_grabber), std::ref(image_sender_people), std::ref(image_sender_screen)));
	// Starting the particle.io thread
	if(p.get("particle"))
		thread_list.push_back(std::thread(sseHandler, std::ref(collector)));
	// Starting the hand detector
	if(p.get("hand"))
		thread_list.push_back(std::thread(handDetector, std::ref(collector), p.get("marker") ? p.getFloat("marker") : 0.033, p.get("calibration"), std::ref(image_sender_table)));
	// Starting the ide logger
	if(p.get("ide"))
		thread_list.push_back(std::thread(ideHandler, std::ref(collector), p.get("mongoose") ? p.getString("mongoose").c_str() : "8081"));
	// Starting audio detector
	if(p.get("audio"))
		thread_list.push_back(std::thread(audioDetector, std::ref(collector)));
	// Starting qr visualization
	if(p.get("qr"))
		thread_list.push_back(std::thread(showQr, session));
	// Starting status visualization
	if(p.get("status"))
		thread_list.push_back(std::thread(drawStatus, std::ref(p)));
	
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
	std::cout << "Io stopped" << std::endl;
	// Stopping Asio aliver
	ws_writer.join();
	std::cout << "writer stopped" << std::endl;
	return 0;
}
