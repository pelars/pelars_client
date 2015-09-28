#include "all.h"

// To stop all the threads if one receives a stop signal
bool to_stop = false;
// Connection status
bool online = true;
// Enable visualization
bool visualization = false;
// Argument parsing structure
struct arguments arguments;
// Mongoose websocket port
const char *  MONGOOSE_PORT= "8081";
// System starting time
const std::string currentDateTimeNow = currentDateTime();

static void sig_handler(int signum)
{
    to_stop = true;
    printf("Received signal %d\n", signum);
}

int main(int argc, char * argv[])
{
  signal(SIGHUP, sig_handler);
  signal(SIGTERM, sig_handler);

  /* Default values. */
  arguments.face = false;
  arguments.hand = false;
  arguments.object = false;
  arguments.visualization = false;
  arguments.particle = false;
  arguments.keylog = false;
  arguments.special = false;
  arguments.template_file = "Not specified";

  // Parse arguments; every option seen by parse_opt will be reflected in arguments. 
  argp_parse (&argp, argc, argv, 0, 0, &arguments);

  visualization = arguments.visualization;
  bool special = arguments.special;

  if(argc == 1)
  {
    std::cout << "please specify at least one sensor source\n" << std::endl;  //TODO add help function
    return -1;
  }

  // Check if the input template list file is correct
  std::ifstream infile(arguments.template_file);
  if(!infile && arguments.object)
  {
    std::cout << "cannot open template list file: " << arguments.template_file << std::endl;
    return -1;
  }
/*
  // Camera capture for face detection
  cv::VideoCapture capture_face(1);
  if(!capture_face.isOpened() && arguments.face)
  {
    std::cout << "Impossible to read from the webcam" << std::endl;
    return -1;
  }
*/
  // Keep aliver
  std::thread ws_writer(asiothreadfx);

  // Start time
  start = orwl_gettime();

  // Kinect Frame acquisition
  KinectManagerExchange * kinect_manager;
  if(arguments.object)
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

  std::cout << "Mongoose websocket started on port 8081" << std::endl;

  std::cout << "Collector endpoint : " << end_point + "collector/" + to_string(session) << std::endl;
  // Check the endpoint string and connect to the session manager
  std::string session_endpoint = end_point + "session/";
  std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;     

  // Websocket manager
  DataWriter collector(end_point + "collector", session);

  // Mongoose websocket listener and message structure
  struct mg_mgr mgr;
  struct mg_connection *nc;
  MiniEncapsule writer(collector, session);
  mg_mgr_init(&mgr, &writer);
  nc = mg_bind(&mgr, "8081", ev_handler);
  mg_set_protocol_http_websocket(nc);
  std::cout << "Mongoose websocket on port 8081" << std::endl;

  // Thread container
  std::vector<std::thread> thread_list;

  // Starting the linemod thread
  if(arguments.object)
    thread_list.push_back(std::thread(linemodf, std::ref(infile), kinect_manager, std::ref(collector)));
  // Starting the face detection thread
  if(arguments.face)
    thread_list.push_back(std::thread(detectFaces, std::ref(collector)));
  // Starting the particle.io thread
  if(arguments.particle)
    thread_list.push_back(std::thread(sseHandler, std::ref(collector)));
  // Starting the hand detector
  if(arguments.hand)
    thread_list.push_back(std::thread(handDetector, std::ref(collector)));
  // Starting the key logger
  if(arguments.keylog)
    thread_list.push_back(std::thread(keyLogger, std::ref(collector), std::ref(io)));
  // Starting the ide logger
  if(arguments.ide)
    thread_list.push_back(std::thread(ideHandler, std::ref(mgr)));
  // Starting audio detector
  if(arguments.audio)
    thread_list.push_back(std::thread(audioDetector, std::ref(collector)));
  
  //If there are no windows wait for Esc to be pressed
  if(!visualization && !special){
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
  std::string tmp;
  if(online)
    tmp = collector.file_name_ + std::string("_backup_") + currentDateTimeNow + collector.file_extention_;
  else
    tmp = collector.file_name_ + std::string("_local_") + currentDateTimeNow + collector.file_extention_;
  std::rename(collector.complete_file_name_.c_str(), tmp.c_str());

  // Terminate everything and exit
  // Close session
  sm.closeSession(session);
  // Stopping the kinect grabber
  if(arguments.object)
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



