#include "hand_detector.h"



void handDetector(DataWriter & websocket, int session)
{

  // OpenCV matrixes to contain the data acquired from the kinect
  cv::Mat depth;

  aruco::MarkerDetector MDetector;
  vector<aruco::Marker> markers;
  if(visualization)
  {
    sleep(1); //else opencv crash for parallel windows instantiation
    cv::namedWindow("hands");
  }

  Json::Value root;
  Json::StyledWriter writer;
  float x, y, z;

  GstreamerGrabber gs_grabber;
  //GstreamerGrabber2 gs_grabber2("/dev/video0", 1920, 1080, true, true);

  IplImage * frame = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 1); //TODO 

  clock_t begin_time = clock();

  float elapsed = 0.0;
  z = 0.0f;  //TODO fix when we have depth

  cv::Mat color;

  while(!to_stop)
  {

    gs_grabber >> frame; 
    
    //gs_grabber2.capture(frame);
    cv::Mat color(frame); 

    MDetector.detect(color, markers);

    elapsed = elapsed + float(clock() - begin_time) / CLOCKS_PER_SEC;

    if(markers.size() > 0)
      for(int i = 0; i < markers.size(); ++i)
      {
        // Get marker position
        markers[i].draw(color, cv::Scalar(0, 0, 255), 2);

        if (elapsed > 100.0){
          elapsed = 0.0;
          begin_time = clock();

          x = markers[i][0].x;
          y = markers[i][0].y;

          root["obj"]["type"] = "hand";
          root["obj"]["id"] = markers[i].id;
          root["obj"]["x"] = x;
          root["obj"]["y"] = y;
          root["obj"]["z"] = z/1000;
          root["obj"]["time"] = deltats(orwl_gettime(), start);
          std::string out_string = writer.write(root);

          // Send the message online and store it offline
          if(online){
            //std::cout << "Hand detector posting data to the server\n " << std::flush;
            io.post( [&websocket, out_string]() {
              websocket.writeData(out_string);
              });
            }

          websocket.writeLocal(out_string);
        }
      }

    if(visualization)
      cv::imshow("hands", color);
      int c = cv::waitKey(1);
      if((char)c == 'q' )
      {
        to_stop = true;
        std::cout << "Stop requested by hand detector" << std::endl;
      }
  }

      //Destroy the window
  cvDestroyWindow("hands");
  return;
}

    