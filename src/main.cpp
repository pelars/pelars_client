#include "mongoose_handler.h"
#include "kinect_grabber.h"
#include "linemod.h"
#include "session_manager.h"
#include "face_detector.h"
#include "sse_handler.h"
#include "hand_detector.h"
#include "opt_parse.h"
#include "key_logger.h"
#include <signal.h>


// To stop all the threads if one receives a stop signal
bool to_stop = false;
bool online = true;
bool visualization = false;
struct arguments arguments;
// Starting time
const std::string currentDateTimeNow = currentDateTime();

void sig_handler(int signum)
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

  // Camera capture for face detection
  cv::VideoCapture capture_face(1);
  if(!capture_face.isOpened() && arguments.face)
  {
    std::cout << "Impossible to read from the webcam" << std::endl;
    return -1;
  }

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

  // Creating a local mongoose server for web debug
  std::cout << "creating moongoose on port 8081" << std::endl;
  struct mg_server * webserver;
  webserver = mg_create_server((void *) "1", ev_handler);
  mg_set_option(webserver, "listening_port", "8081");
  std::thread mg_thread(serve, webserver);
  std::cout << "moongoose ready" << std::endl;

  // Check the endpoint string and connect to the collector
  std::string end_point = "http://pelars.sssup.it:8080/pelars/";
  end_point = end_point.back() == '/' ? end_point : end_point + "/";
  std::cout << "WebServer endpoint : " << end_point << std::endl;

  //Creating a Session Manager and getting a newsession ID
  SessionManager sm(end_point);
  int session = sm.getNewSession();
  
  std::cout << "Collector endpoint : " << end_point + "collector/" + to_string(session) << std::endl;

  // Check the endpoint string and connect to the session manager
  std::string session_endpoint = end_point + "session/";
  std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;     

  // Websocket manager
  DataWriter collector(end_point + "collector", session);

  // Thread container
  std::vector<std::thread> thread_list;
  
  // Starting the linemod thread
  if(arguments.object)
    thread_list.push_back(std::thread(linemodf, std::ref(infile), kinect_manager, std::ref(collector), session));
  // Starting the face detection thread
  if(arguments.face)
    thread_list.push_back(std::thread(detectFaces, std::ref(collector), std::ref(capture_face), session));
  // Starting the particle.io thread
  if(arguments.particle)
    thread_list.push_back(std::thread(sseHandler, std::ref(collector), session));
  // Starting the hand detector
  if(arguments.hand)
    thread_list.push_back(std::thread(handDetector, std::ref(collector), session));
  // Starting the key logger
  if(arguments.keylog)
    thread_list.push_back(std::thread(keyLogger, std::ref(collector), session, std::ref(io)));
  
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
  // Stopping mongoose
  mg_thread_stop = true;
  mg_thread.join();
  //std::cout << "Mongoose stopped" << std::endl;
  // Stopped io service 
  io.stop();
  // Stopping Asio aliver
  ws_writer.join();
  std::cout << "IO stopped" << std::endl; 

  return 0;
}



