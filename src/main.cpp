#include "all.h"

std::mutex synchronizer;
ParameterServer parameter_server;
ParameterSpace parameters(parameter_server, "pelars");

// Triggers
ChannelWrapper<Trigger> pc_trigger(to_stop, 3);

// Webcam frames message channels
ChannelWrapper<ImageFrame> pc_webcam(to_stop, 3);

// Kinect frames message channels
ChannelWrapper<ImageFrame> pc_kinect(to_stop, 3);

// Screen frames message channel
ChannelWrapper<ImageFrame> pc_screen(to_stop, 3);

int main(int argc, char * argv[])
{

	// Signal handlers
	signal(SIGHUP, sig_handler);
	signal(SIGTERM, sig_handler);

	static unsigned int trigger_time = 60;
	static unsigned int scree_shotter_timer_ms = 100;
	int session;

	// Thread container
	std::vector<std::thread> thread_list;

	// Parse input arguments
	Parser p(argc, argv);
	if(p.get("help") || argc == 1){
		p.printHelp();
		return 0;
	}

	bool delete_h264 = p.get("h264") ? false : true;
	bool store_video = p.get("video");

	unsigned int width = p.get("width") ? p.getInt("width") : 1920;
	unsigned int height = p.get("height") ? p.getInt("height") : 1080;

	// Keep alive for Asio
	std::thread ws_writer(asiothreadfx);

	// Check the endpoint string and connect to the collector
	std::string end_point = p.get("server") ? p.getString("server") : "http://pelars.sssup.it/pelars/";
	end_point = end_point.back() == '/' ? end_point : end_point + "/";
	std::cout << "WebServer endpoint : " << end_point << std::endl;

#ifdef HAS_FREENECT2
	K2G::Processor processor = static_cast<K2G::Processor>(p.get("processor") ? p.getInt("processor") : 1);
#endif
	
	if(p.get("upload")){
		std::string upload = p.getString("upload");
		int error = uploadData(upload, end_point, p.get("session") ? p.getInt("session") : 0);
		io.stop();
		ws_writer.join();
		return !error;
	}

	// Check if video device is different than /dev/video0
	int face_camera_id = p.get("face_camera") ? p.getInt("face_camera") : 0;
	int hand_camera_id = p.get("hand_camera") ? p.getInt("hand_camera") : 1;
	float marker_size = p.get("marker") ? p.getFloat("marker") : 0.07;

	if(p.get("marker_id")){
		show_markers(face_camera_id, marker_size, pc_webcam.getNewChannel());
		io.stop();
		ws_writer.join();
		return 0;
	}

	// Calibrate the cameras and exit
	if(p.get("calibration")){
#if defined(HAS_ARUCO) && defined(HAS_FREENECT2)		
		calibration(face_camera_id, hand_camera_id, marker_size, p.get("C920"), processor, p.get("no_calib_video"));
		io.stop();
		ws_writer.join();
#endif
		return 0;
	}

	// Creating a Session Manager and getting a newsession ID
	SessionManager sm(end_point, p.get("test"));
	sm.login();
	std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();
	
	if(!p.get("session"))
		session = sm.getNewSession((double)std::chrono::duration_cast<std::chrono::milliseconds>(start.time_since_epoch()).count());
	else{
		session = p.getInt("session");
		std::cout << "Using session " << session << std::endl;
	}
	std::string token = sm.getToken(); 

	// Check the endpoint string and connect to the session manager
	std::cout << "Collector endpoint : " << end_point + "collector/" + std::to_string(session) << std::endl;

	// Websocket manager
	DataWriter collector(end_point + "collector", session);
	DataWriter alive_socket(end_point + "aliver", session); 
	std::cout << "opened aliver on " + end_point + std::to_string(session) << std::endl;
	//Wait for websockets to connect 
	sleep(1);


	visualization = p.get("visualization");

	// Send the two calibration matrixes. Need to sleep to give the websocket time to connect :/
	if(sendCalibration(collector) != 0){
		std::cout << "error sending the calibration. Did you calibrate the cameras with -c ?" << std::endl;
	}

	//Image sender for the screenshots
	thread_list.push_back(std::thread(sendImage, session, std::ref(end_point), std::ref(token), pc_screen.getNewChannel(), 
									  pc_trigger.getNewChannel(), false));

	//Image grabber for the screenshots
	thread_list.push_back(std::thread(screenShotter, std::ref(pc_screen), scree_shotter_timer_ms));


	// Starting the face detection thread
	if(p.get("face") || p.get("default")){
		thread_list.push_back(std::thread(webcamPublisher, face_camera_id, std::ref(pc_webcam), width, height, std::ref(collector)));
		thread_list.push_back(std::thread(detectFaces, std::ref(collector), pc_webcam.getNewChannel(), p.get("video")));
		thread_list.push_back(std::thread(sendImage, session, std::ref(end_point), 
			                              std::ref(token), pc_webcam.getNewChannel(), pc_trigger.getNewChannel(), false));
		if(store_video){
			thread_list.push_back(std::thread(saveVideo, session, pc_webcam.getNewChannel(), delete_h264));
		}
	}

	// Starting the kinect grabber
	if(p.get("hand") || p.get("default")){
		
		thread_list.push_back(std::thread(kinect2publisher, processor, std::ref(pc_kinect), std::ref(collector)));
		thread_list.push_back(std::thread(sendImage, session, std::ref(end_point), 
			                              std::ref(token), pc_kinect.getNewChannel(), pc_trigger.getNewChannel(), false));
#ifdef HAS_ARUCO
		thread_list.push_back(std::thread(handDetector, std::ref(collector), p.get("marker") ? p.getFloat("marker") : 0.035, 
								          pc_kinect.getNewChannel(), p.get("C920"), hand_camera_id));
#endif	
		if(store_video){
			thread_list.push_back(std::thread(saveVideo, session, pc_kinect.getNewChannel(), delete_h264));
		}
	}

	thread_list.push_back(std::thread(sendTrigger, std::ref(pc_trigger), trigger_time));

#ifdef HAS_CURL
	// Starting the particle.io thread 	
	if(p.get("particle"))
		thread_list.push_back(std::thread(sseHandler, std::ref(collector)));
#endif

	// Starting the ide logger
	if(p.get("ide") || p.get("default")){
		IdeTrigger ide_trigger(pc_trigger, &collector);
		thread_list.push_back(std::thread(ideHandler, std::ref(ide_trigger), 
			                              p.get("mongoose") ? p.getString("mongoose").c_str() : "8081", "8082"));
	}
	
	// Starting audio detector
	if(p.get("audio") || p.get("default"))
		thread_list.push_back(std::thread(audioDetector, std::ref(collector)));

	// Starting qr visualization
	if(p.get("qr") || p.get("default"))
		thread_list.push_back(std::thread(showQr, session));
	
	// Starting status visualization
	if(p.get("status") || p.get("default"))
		thread_list.push_back(std::thread(drawStatus, std::ref(p)));

	// Keep alive on server for status update
	thread_list.push_back(std::thread(keep_alive, std::ref(alive_socket)));
	if(p.get("video_session"))
		thread_list.push_back(std::thread(sessionWriter, session));
	//std::thread audio_recorder = std::thread(audioRecorder, session);
	
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

	// Stopping the websocket
	collector.stop();
	std::cout << "Connection to Collector closed" << std::endl;
	// Stopped io service 
	io.stop();
	std::cout << "Io stopped" << std::endl;
	// Stopping Asio aliver
	ws_writer.join();
	std::cout << "writer stopped" << std::endl;

	//parameter_server.dump("pelars_config.yaml");

	kill(0, SIGINT);
	return 0;
}
