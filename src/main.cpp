#include "all.h"

int main(int argc, char * argv[])
{
	// Signal handlers
	signal(SIGHUP, sig_handler);
	signal(SIGTERM, sig_handler);

	// Parse input arguments
	Parser p(argc, argv);
	if(p.get("help") || argc == 1){
		p.printHelp();
		return 0;
	}

	// Keep aliver
	std::thread ws_writer(asiothreadfx);

	// Check the endpoint string and connect to the collector
	std::string end_point = p.get("Server") ? p.getString("Server") : "http://pelars.sssup.it/pelars/";
	end_point = end_point.back() == '/' ? end_point : end_point + "/";
	std::cout << "WebServer endpoint : " << end_point << std::endl;

	K2G::Processor processor = (K2G::Processor)(p.get("processor") ? p.getInt("processor") : 1);
	
	if(p.get("upload")){
		int error;
		error = uploadData(p.getString("upload"), end_point, p.get("session") ? p.getInt("session") : 0);
		io.stop();
		ws_writer.join();
		return !error;
	}

	// Check if video device is different than /dev/video0
	int face_camera_id = 0;
	if(p.get("face_camera"))
		face_camera_id = p.getInt("face_camera");

	// Calibrate the cameras and exit
	if(p.get("calibration")){
		calibration(face_camera_id, p.get("marker") ? p.getFloat("marker") : 0.07);
		io.stop();
		ws_writer.join();
		return 0;
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

	// Check the endpoint string and connect to the session manager
	std::cout << "Collector endpoint : " << end_point + "collector/" + to_string(session) << std::endl;
	std::string session_endpoint = end_point + "session/";
	std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;    

	// Websocket manager
	DataWriter collector(end_point + "collector", session); 

	visualization = p.get("visualization");


	// Image grabber
	ImageSender image_sender_table(session, end_point, token);
	ImageSender image_sender_people(session, end_point, token);
	ImageSender image_sender_screen(session, end_point, token);

	// Screen grabber
	ScreenGrabber screen_grabber;

	// Kinect Frame acquisition and check if the input template list file is correct
	std::ifstream infile;
	KinectManagerExchange * kinect_manager;
	if(p.get("object"))
	{
		kinect_manager = new KinectManagerExchange();
		if(*kinect_manager)
			kinect_manager->start();
		else{
			return -1;
		}

		infile.open(p.getString("object"));
		if(!infile)
		{
			std::cout << "cannot open template list file: " << p.getString("object") << std::endl;
			p.printHelp();
			return -1;
		}
	}

	// Send the two calibration matrixes. Need to sleep to give the websocket time to connect :/
	sleep(1);
	if(sendCalibration(collector) != 0){
		std::cout << "error sending the calibration. Did you calibrate the cameras with -c ?" << std::endl;
	}

	// Thread container
	std::vector<std::thread> thread_list;

	cout << '\a' << std::flush;

	// Starting the linemod thread
	if(p.get("object"))
		thread_list.push_back(std::thread(linemodf, std::ref(infile), kinect_manager, std::ref(collector)));
	// Starting the face detection thread
	if(p.get("face"))
		thread_list.push_back(std::thread(detectFaces, std::ref(collector), std::ref(screen_grabber), std::ref(image_sender_people), std::ref(image_sender_screen), face_camera_id));
	// Starting the particle.io thread
	if(p.get("particle"))
		thread_list.push_back(std::thread(sseHandler, std::ref(collector)));
	// Starting the hand detector
	if(p.get("hand"))
		thread_list.push_back(std::thread(handDetector, std::ref(collector), p.get("marker") ? p.getFloat("marker") : 0.033, std::ref(image_sender_table), processor));
	// Starting the ide logger
	if(p.get("ide"))
		thread_list.push_back(std::thread(ideHandler, std::ref(collector), p.get("mongoose") ? p.getString("mongoose").c_str() : "8081", "8082"));
	// Starting audio detector
	if(p.get("audio"))
		thread_list.push_back(std::thread(audioDetector, std::ref(collector)));
	// Starting qr visualization
	if(p.get("qr"))
		thread_list.push_back(std::thread(showQr, session));
	// Starting status visualization
	if(p.get("status"))
		thread_list.push_back(std::thread(drawStatus, std::ref(p)));
	// Keep alive on server for status update
	thread_list.push_back(std::thread(keep_alive, session, end_point + "aliver"));
	
	//If there are no windows wait for Esc to be pressed
	checkEscape(visualization, p.get("special"));

	// Wait for the termination of all threads
	for(auto & thread : thread_list)
		thread.join();
	
	// Create a local file for data acquisition and backup
	std::string tmp = collector.file_name_ + std::string(online ? "_backup_" : "_local_") + currentDateTime() + collector.file_extention_;
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
