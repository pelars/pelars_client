
#include "mongoose_handler.h"
#include "kinect_grabber.h"
#include "linemod.h"
#include <thread>

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

  std::vector<std::thread> thread_list(1);

  //Keep aliver
  std::thread ws_writer(asiothreadfx);

  // Start time
  start = orwl_gettime();

  // Kinect Frame acquisition
  KinectManagerExchange kme;
  kme.start();

  // Check the endpoint string and connect to the collector
  // TODO if connection fails exit
  std::string end_point = argc < 3 ? "http://127.0.0.1:8080/pelars/" : argv[2];  //http://10.100.34.226:8080/pelars/
  end_point = end_point.back() == '/' ? end_point : end_point + "/";
  DataWriter collector(end_point + "Collector");
  std::cout << "WebServer endpoint : " << end_point << std::endl;
  std::cout << "Collector endpoint : " << end_point + "Collector" << std::endl;

  // Check the endpoint string and connect to the session manager
  // TODO if connection fails exit
  std::string session_endpoint = end_point + "Sessionmanager";
  std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;;
  std::string session_endpoint_data;

  // Prepare session endpoint and initial data, connect to the remote session manager
  
  boost::network::http::client client;
  boost::network::http::client::response response;
  std::string type, teacher_name, institution_name, institution_address;
  std::stringstream string_stream;
  std::string session_manager_response;
  int session;
  srand (time(NULL));

  // Example data (has to be acquireds somehow)
  type = "open";
  teacher_name = "test_name";
  institution_name = "test_name_2";
  institution_address = "test_address";
  session_endpoint_data="?type="+type+"&teacher_name="+teacher_name+"&institution_name="+institution_name+"&institution_address="+institution_address+"&session_id=";

  // Continue sending possible session id's until a good one is found and the session manager gives a positive answer
  // TODO Make some other check on the session manager liveness and on the time spent asking for a session
  do
  {
    session = rand();
    std::cout << "asking for session id " << session << std::endl;

    boost::network::http::client::request request(session_endpoint + session_endpoint_data + std::to_string(session));
    response = client.get(request);
    string_stream << body(response);
    session_manager_response = string_stream.str();

  }while(!session_manager_response.compare("open"));

  // Clear the string stream
  string_stream.str(std::string());


  // Creating a local mongoose server for web debug
  std::cout << "openin moongoose for debug on port 8081" << std::endl;
  struct mg_server * webserver;
  webserver = mg_create_server((void *) "1", ev_handler);
  mg_set_option(webserver, "listening_port", "8081");
  std::thread mg_thread(serve, webserver);
  std::cout << "moongoose ready" << std::endl;


  thread_list[1] = std::thread(linemodf, std::ref(infile), std::ref(kme), std::ref(collector), session);

  thread_list[1].join();

  // Close the session and exit
  // Preaparing data to close the session
  type = "close";
  std::cout << "closing session  " << session << std::endl;
  session_endpoint_data="?type="+type+"&teacher_name="+teacher_name+"&institution_name="+institution_name+"&institution_address="+institution_address+"&session_id="+std::to_string(session);

  //Sending data to close the session with the session manager
  boost::network::http::client::request request(session_endpoint + session_endpoint_data);
  response = client.get(request);
  string_stream << body(response);
  std::string client_response = string_stream.str();
  std::cout << "Session manager resonse " << client_response << std::endl;

  // Terminate everything and exit
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