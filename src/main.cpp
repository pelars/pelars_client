
#include "mongoose_handler.h"
#include "kinect_grabber.h"
#include "linemod.h"
#include "session_manager.h"
#include "face_detector.h"
#include "sse_handler.h"
#include "hand_detector.h"


// To stop all the threads if one receives a stop signal
bool to_stop = false;
bool online = true;
const std::string currentDateTimeNow = currentDateTime();

int main(int argc, char * argv[])
{
  // Check correct number of input arguments
  if(argc < 2)
  {
    std::cout << "use template file list and the websocket endpoint as input parameter" << std::endl;
    return -1;
  }

  // Check if the input template list file is correct
  std::ifstream infile(argv[1]);
  if(!infile)
  {
    std::cout << "cannot open template list file " << argv[1] << std::endl;
    return -1;
  }

  // Standard vector containing the different threads
  std::vector<std::thread> thread_list(4);

  // Keep aliver
  std::thread ws_writer(asiothreadfx);

  // Start time
  start = orwl_gettime();

  // Kinect Frame acquisition
  KinectManagerExchange kme;
  kme.start();

  // Camera capture
  cv::VideoCapture capture(0);

  // Check the endpoint string and connect to the collector
  // TODO if connection fails exit
  std::string end_point = "http://10.100.35.191:8080/pelars/";
  //std::string end_point = "http://10.100.35.191:8080/pelars/";

  end_point = end_point.back() == '/' ? end_point : end_point + "/";
  
  std::cout << "WebServer endpoint : " << end_point << std::endl;
  std::cout << "Collector endpoint : " << end_point + "collector" << std::endl;

  // Check the endpoint string and connect to the session manager
  std::string session_endpoint = end_point + "session/";
  std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;

  //Creating a Session Manager and getting a newsession ID
  SessionManager sm(end_point);
  int session = sm.getNewSession();     

  // Websocket manager
  DataWriter collector(end_point + "collector", session);

  // Creating a local mongoose server for web debug
  /*
  std::cout << "Opening moongoose for debug on port 8081" << std::endl;
  struct mg_server * webserver;
  webserver = mg_create_server((void *) "1", ev_handler);
  mg_set_option(webserver, "listening_port", "8081");
  std::thread mg_thread(serve, webserver);
  std::cout << "\tMoongoose ready" << std::endl;
  */

  // Starting the linemod thread
  thread_list[0] = std::thread(linemodf, std::ref(infile), std::ref(kme), std::ref(collector), session);
  thread_list[1] = std::thread(detectFaces, std::ref(collector), std::ref(capture), session);
  thread_list[2] = std::thread(sseHandler, std::ref(collector), session);
  thread_list[3] = std::thread(handDetector, std::ref(kme), std::ref(collector), std::ref(capture), session);

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
  kme.stop();
  // Stopping the websocket
  collector.stop();
  std::cout << "Connection to Collector closed" << std::endl;
  // Stopping mongoose
  //mg_thread_stop = true;
  //mg_thread.join();
  //std::cout << "Mongoose stopped" << std::endl;
  // Stopped io service 
  io.stop();
  // Stopping Asio aliver
  ws_writer.join();
  std::cout << "IO stopped" << std::endl; 

  return 0;
  
}