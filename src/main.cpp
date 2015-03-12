
#include <json/json.h>
#include "alttime.h"
#include "data_writer.h"
#include "ompparallelFor.h"
#include "mongoose_handler.h"
#include "kinect_grabber.h"
#include "linemod.h"


int main(int argc, char * argv[])
{
  // Check correct number of input arguments
  if(argc < 2)
  {
    std::cout << "use template file list and the websocket endpoint as input parameter" << std::endl;
    return -1;
  }

  // Check if the input teomplate list file is correct
  std::ifstream infile(argv[1]);
  if(!infile)
  {
    std::cout << "cannot open template list file " << argv[1] << std::endl;
    return -1;
  }

  // Kinect Frame acquisition
  KinectManagerExchange kme;
  kme.start();

  //Keep aliver
  std::thread ws_writer(asiothreadfx);

  // Start time
  start = orwl_gettime();

  // Some matching parameters for linemod
  short matching_threshold = 85;
  short learning_lower_bound = 90;
  short learning_upper_bound = 95;

  //Kinect v1 intrinsic parameters
  const double focal_x_depth = 5.9421434211923247e+02;
  const double focal_y_depth = 5.9104053696870778e+02;
  const double center_x_depth = 3.3930780975300314e+02;
  const double center_y_depth = 2.4273913761751615e+02;

  // Check the endpoint string and connect to the collector
  // TODO if connection fails exit
  std::string end_point = argc < 3 ? "http://127.0.0.1:8080/pelars/" : argv[2];  //http://10.100.34.226:8080/pelars/
  end_point = end_point.back() == '/' ? end_point : end_point + "/";
  DataWriter websocket(end_point + "Collector");
  std::cout << "WebServer endpoint : " << end_point << std::endl;
  std::cout << "Collector endpoint : " << end_point + "Collector" << std::endl;

  // Creating a local mongoose server for web debug
  std::cout << "openin moongoose for debug on port 8081" << std::endl;
  struct mg_server * webserver;
  webserver = mg_create_server((void *) "1", ev_handler);
  mg_set_option(webserver, "listening_port", "8081");
  std::thread mg_thread(serve, webserver);
  std::cout << "moongoose ready" << std::endl;

  // Initialize an OpenCV window
  cv::namedWindow("color");

  // Initialize LINEMOD data structures
  std::vector<cv::Ptr<cv::linemod::Detector>> detectors;
  std::vector<int> num_modalities_vector;

  // Load all the templates listed in the input file
  std::string path;
  short i = 0;
  while (infile >> path)
  {
    std::cout << "loading " << path << std::endl;
    detectors.push_back(readLinemod(path));
    std::cout <<"Loaded " << path.c_str() <<" with " << detectors[i]->numClasses() << " classes and "<< detectors[i]->numTemplates() << " templates" << std::endl;
    i++;
  }

  // Check if any template is loaded
  short template_num = detectors.size();
  if(!template_num){
    std::cout << "no models loaded" << std::endl;
    return -1;
  }

  // Preapare modalities vector
  for(int i = 0; i < template_num; ++i)
    num_modalities_vector.push_back( (int)detectors[i]->getModalities().size());
  std::vector<std::vector<cv::linemod::Match>> matches_vector(template_num);


  // Prepare session endpoint and initial data, connect to the remote session manager
  std::string session_endpoint = end_point + "Sessionmanager";
  std::cout << "Session Manager endpoint : " << session_endpoint  << std::endl;;
  std::string session_endpoint_data;
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

  // Creating a thread pool to detect templates in parallel
  ThreadPool pool(std::thread::hardware_concurrency());

  // OpenCV matrixes to contain the data acquired from the kinect
  cv::Mat color, depth;

  // Timing variables
  double matching_time;
  struct timespec begin, end;


  // Linemod variables
  std::vector<cv::Mat> sources;
  cv::Mat display;
  double min;
  double max;

  //Main loop. Executes until 'q' is pressed or there is an error with the kinect acquisition.
  for(;;)
  {
    std::cout << "getting data" << std::endl;
    // Acquire depth and color images from the kinect and prepare them for linemod
    if(!kme.get(color, depth))
    {
      std::cout << "failed to fetch data from the kinect\n";
      break;
    }
    std::cout << "got data" << std::endl;
    sources.push_back(color);
    sources.push_back(depth);
    display = color.clone();

    // Get start time
    begin = orwl_gettime();

    // Prepare structures for the parallel template matching
    std::vector<std::vector<result_t> > results(template_num);
    std::vector<std::string> class_ids;

    // Lambda function to match the tamplates
    auto myFunc = [=, &matches_vector, &websocket, &detectors, &class_ids, &results](unsigned int i) {
      int num_classes = detectors[i]->numClasses();        
      int classes_visited = 0;
      std::vector<cv::Mat> quantized_images;
      std::set<std::string> visited;
      matches_vector[i].clear();

      detectors[i]->match(sources, (float)matching_threshold, matches_vector[i], class_ids, quantized_images);

      for (int j = 0; (j < (int)matches_vector[i].size()) && classes_visited < num_classes; ++j)
      {
        cv::linemod::Match m = matches_vector[i][j];
        if (visited.insert(m.class_id).second)
        {
          ++classes_visited;

          if(m.similarity >= 88.0 )
          {
            // mutex
            //results
            result_t t;
            t.m = m;
            t.end = orwl_gettime();
            results[i].push_back(t);
          }
          else
            std::cout << "bad quality for obj: " << i << " with similarity: " << m.similarity << std::endl;
        }
      }
    };

    //parallelFor(template_num, myFunc);
    parallelFor(pool, template_num, myFunc);

    //timing variable
    double elapsed;

    for(int i = 0; i < template_num; i++)
    {
      for(int j = 0; j < results[i].size(); j++)
      {
        // Get results and display them
        auto & m = results[i][j].m;
        drawResponse(detectors[i]->getTemplates(m.class_id, m.template_id), num_modalities_vector[i], display, cv::Point(m.x, m.y), detectors[i]->getT(0),i);
        display.at<cv::Vec3b>(m.y, m.x) = cv::Vec3b(0, 0, 255);

        // Preapare JSON message to send to the Collector
        //TODO Parallelize
        Json::Value root;
        Json::Value pos;
        Json::StyledWriter writer;

        //Elapsed time from process start
        elapsed = deltats(results[i][j].end, start);

        // Template data
        int dz = depth.at<uint16_t>(m.y,m.x);
        pos[0] = (m.x - center_x_depth) * dz / focal_x_depth;
        pos[1] = (m.x - center_x_depth) * dz / focal_x_depth;
        pos[2] = dz;

        // Json message
        root["obj"]["type"] = "object";
        root["obj"]["session"] = session;
        root["obj"]["id"] = j;
        root["obj"]["x"] = m.x; // (m.x - center_x_depth) * dz / focal_x_depth;
        root["obj"]["y"] = m.y; // (m.y - center_y_depth) * dz / focal_y_depth;
        root["obj"]["z"] = dz;
        root["obj"]["pos"] = pos; // maybe not and one at level of ...
        root["obj"]["time"] = elapsed; // session relative

        //Send message
        std::string out_string = writer.write(root);
        io.post( [&websocket, out_string]() {
             websocket.writeData(out_string);
         });
      }
    }

    //Total matching time 
    end = orwl_gettime();
    matching_time = (end.tv_sec - begin.tv_sec);
    matching_time += (end.tv_nsec - begin.tv_nsec) / 1000000000.0;
    std::cout << "total matching time " << matching_time << std::endl;
    
    // Get min and max values of the depth map and convert it to grayscale
    cv::minMaxIdx(depth, &min, &max);
    cv::Mat adjMap;
    depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min); 

    // Display color and depth
    cv::imshow("adjMap", adjMap);
    cv::imshow("color", display);

    // Check if any key is pressed and in case process input
    char key = (char)cvWaitKey(10);
    if( key == 'q' )
        break;
    switch (key)
    {
      case '[':
        // decrement threshold
        matching_threshold = std::max(matching_threshold - 1, -100);
        printf("New threshold: %d\n", matching_threshold);
        break;
      case ']':
        // increment threshold
        matching_threshold = std::min(matching_threshold + 1, +100);
        printf("New threshold: %d\n", matching_threshold);
        break;
      default:
        ;
    }
  // Clear the depth and color input in order to grab a new one
  sources.clear();
  }

// Close the session and exit
exitloop:
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
niceexit:
  // Stopping the kinect grabber
  kme.stop();
  // Stopping the websocket
  websocket.stop();
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

}