#include "all.h"
#include "screen_grabber.h"

std::mutex synchronizer;

int main(int argc, char * argv[])
{

	// Signal handlers
	signal(SIGHUP, sig_handler);
	signal(SIGTERM, sig_handler);
	initTermination();

	static int trigger_time = 60;


	// Parse input arguments
	Parser p(argc, argv);
	if(p.get("help") || argc == 1){
		p.printHelp();
		return 0;
	}

	// Keep alive for Asio
	std::thread ws_writer(asiothreadfx);

	// Check the endpoint string and connect to the collector
	std::string end_point = p.get("server") ? p.getString("server") : "http://pelars.sssup.it/pelars/";
	end_point = end_point.back() == '/' ? end_point : end_point + "/";
	std::cout << "WebServer endpoint : " << end_point << std::endl;

#ifdef HAS_FREENECT2
	K2G::Processor processor = static_cast<K2G::Processor>(p.get("processor") ? p.getInt("processor") : 2);
#endif
	
	if(p.get("upload")){
		int error;
		std::string file_name = p.getString("upload");
		error = uploadData(file_name, end_point, p.get("session") ? p.getInt("session") : 0);
		io.stop();
		ws_writer.join();
		return !error;
	}

	// Check if video device is different than /dev/video0
	int face_camera_id = 0;
	int hand_camera_id = 1;
	if(p.get("face_camera"))
		face_camera_id = p.getInt("face_camera");

	if(p.get("hand_camera"))
		hand_camera_id = p.getInt("hand_camera");

	float marker_size = p.get("marker") ? p.getFloat("marker") : 0.07;

	if(p.get("marker_id")){
		show_markers(face_camera_id, marker_size);
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
	int session;
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
	std::string session_endpoint = end_point + "session/";
	std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;    

	// Websocket manager
	DataWriter collector(end_point + "collector", session); 
	DataWriter alive_socket(end_point + "aliver", session); 
	std::cout << "opened aliver on " + end_point + std::to_string(session) << std::endl;

	visualization = p.get("visualization");

	// Send the two calibration matrixes. Need to sleep to give the websocket time to connect :/
	//sleep(1);
	if(sendCalibration(collector) != 0){
		std::cout << "error sending the calibration. Did you calibrate the cameras with -c ?" << std::endl;
	}

	// Thread container
	std::vector<std::thread> thread_list;

	// Webcam frames message channels
	std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> pc_webcam = makeChannel<ImageFrame>(3, to_stop, 3);

	// Kinect frames message channels
	std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> pc_kinect = makeChannel<ImageFrame>(3, to_stop, 3);

	// Triggers
	std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>> pc_trigger = makeChannel<Trigger>(3, to_stop, 3);


	thread_list.push_back(std::thread(sendTrigger, std::ref(pc_trigger), trigger_time));

	// Screen frames message channel
	std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> pc_screen = makeChannel<ImageFrame>(1, to_stop, 3);

	thread_list.push_back(std::thread(screenShotter, std::ref(pc_screen)));
	thread_list.push_back(std::thread(sendImage, session, std::ref(end_point), std::ref(token), pc_screen[0], pc_trigger[2], false));


	// Starting the face detection thread
	if(p.get("face") || p.get("default")){
		thread_list.push_back(std::thread(webcamPublisher, face_camera_id, std::ref(pc_webcam)));
		thread_list.push_back(std::thread(detectFaces, std::ref(collector), pc_webcam[0], p.get("video")));
		thread_list.push_back(std::thread(sendImage, session, std::ref(end_point), 
			                              std::ref(token), pc_webcam[1], pc_trigger[1], false));
		if(p.get("video")){
			thread_list.push_back(std::thread(saveVideo, session, pc_webcam[2]));
		}
	}

	// Starting the kinect grabber
	if(p.get("hand") || p.get("default")){
		
		thread_list.push_back(std::thread(kinect2publisher, processor, std::ref(pc_kinect)));
		thread_list.push_back(std::thread(sendImage, session, std::ref(end_point), 
			                              std::ref(token), pc_kinect[1], pc_trigger[0], false));
#ifdef HAS_ARUCO
		thread_list.push_back(std::thread(handDetector, std::ref(collector), p.get("marker") ? p.getFloat("marker") : 0.035, 
								pc_kinect[0], p.get("C920"), hand_camera_id));
#endif	

		if(p.get("video")){
			thread_list.push_back(std::thread(saveVideo, session, pc_kinect[2]));
		}
	}

	

#ifdef HAS_CURL
	// Starting the particle.io thread 	
	if(p.get("particle"))
		thread_list.push_back(std::thread(sseHandler, std::ref(collector)));
#endif

	// Starting the ide logger
	if(p.get("ide") || p.get("default"))
		thread_list.push_back(std::thread(ideHandler, std::ref(collector), p.get("mongoose") ? p.getString("mongoose").c_str() : "8081", "8082"));
	
	// Starting audio detector
	if(p.get("audio"))
		thread_list.push_back(std::thread(audioDetector, std::ref(collector)));

	// Starting qr visualization
	if(p.get("qr") || p.get("default"))
		thread_list.push_back(std::thread(showQr, session));
	
	// Starting status visualization
	if(p.get("status"))
		thread_list.push_back(std::thread(drawStatus, std::ref(p)));

	// Keep alive on server for status update
	thread_list.push_back(std::thread(keep_alive, std::ref(alive_socket)));
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

	kill(0, SIGINT);
	return 0;
}
