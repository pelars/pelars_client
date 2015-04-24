
#include "mongoose_handler.h"
#include "kinect_grabber.h"
#include "linemod.h"
#include "session_manager.h"
#include "face_detector.h"
#include <thread>

// To stop all the threads if one receives a stop signal
bool to_stop = false;
bool online = true;

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
  std::vector<std::thread> thread_list(2);

  // Keep aliver
  std::thread ws_writer(asiothreadfx);

  // Start time
  start = orwl_gettime();

  // Kinect Frame acquisition
  KinectManagerExchange kme;
  kme.start();

  // Check the endpoint string and connect to the collector
  // TODO if connection fails exit
  std::string end_point = argc < 3 ? "http://127.0.0.1:8080/pelars/" : argv[2];  //http://10.100.34.226:8080/pelars/
  //std::string end_point = argc < 3 ? "http://127.0.0.1:8080/networkmanager/" : argv[2];
  end_point = end_point.back() == '/' ? end_point : end_point + "/";
  
  std::cout << "WebServer endpoint : " << end_point << std::endl;
  std::cout << "Collector endpoint : " << end_point + "Collector" << std::endl;
  //std::cout << "Collector endpoint : " << end_point + "Localcollector" << std::endl;

  // Check the endpoint string and connect to the session manager
  // TODO if connection fails exit
  std::string session_endpoint = end_point + "session/";
  //std::string session_endpoint = "http://127.0.0.1:8080/networkmanager/Localsessionmanager";
  std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;

  //Creating a Session Manager and getting a newsession ID
  SessionManager sm(session_endpoint);
  int session = sm.getNewSession();

  DataWriter collector(end_point + "Collector", session);
  //DataWriter collector(end_point + "Localcollector");

  // Creating a local mongoose server for web debug
  std::cout << "opening moongoose for debug on port 8081" << std::endl;
  struct mg_server * webserver;
  webserver = mg_create_server((void *) "1", ev_handler);
  mg_set_option(webserver, "listening_port", "8081");
  std::thread mg_thread(serve, webserver);
  std::cout << "moongoose ready" << std::endl;

  // Starting the linemod thread
  thread_list[0] = std::thread(linemodf, std::ref(infile), std::ref(kme), std::ref(collector), session);
  thread_list[1] = std::thread(detectFaces, std::ref(collector), session);

  // Wait for the termination of all threads
  for(auto &thread : thread_list)
    thread.join();


  std::string tmp;
  if(online)
    tmp = collector.file_name_ + std::string("_backup_") + collector.file_extention_;
  else
    tmp = collector.file_name_ + std::string("_local_") + collector.file_extention_;

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
  mg_thread_stop = true;
  mg_thread.join();
  std::cout << "Mongoose stopped" << std::endl;
  // Stopped io service 
  io.stop();
  // Stopping Asio aliver
  ws_writer.join();
  std::cout << "IO stopped" << std::endl; 

  return 0;
  
}