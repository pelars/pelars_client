#include "hand_detector.h"

void handDetector(DataWriter & websocket)
{

  // OpenCV matrixes to contain the data acquired from the kinect
  cv::Mat depth;

  aruco::MarkerDetector MDetector;
  vector<aruco::Marker> markers;
  if(visualization)
  {
    //sleep(1); //else opencv crash for parallel windows instantiation
    cv::namedWindow("hands");
  }

  Json::Value root;
  Json::StyledWriter writer;
  float x, y, z;

  //GstreamerGrabber gs_grabber(1920, 1080, 1);
  //GstreamerGrabber2 gs_grabber2("/dev/video0", 1920, 1080, true, false);
  //IplImage * frame = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 1); //TODO 
  //Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("../../data/calibration/rgb_calibration.yaml", "../../data/calibration/depth_calibration.yaml", "../../data/calibration/pose_calibration.yaml");
  
  K2G k2g(OPENGL);

  clock_t begin_time = clock();
  float elapsed = 0.0;
  z = 0.0f;

  cv::Mat camera_parameters = cv::Mat::eye(3, 3, CV_32F);
  camera_parameters.at<float>(0,0) = k2g.getRgbParameters().fx; 
  camera_parameters.at<float>(1,1) = k2g.getRgbParameters().fy; 
  camera_parameters.at<float>(0,2) = k2g.getRgbParameters().cx; 
  camera_parameters.at<float>(1,2) = k2g.getRgbParameters().cy;
  cv::Mat distortion = cv::Mat(1, 4, 0);
  

  aruco::CameraParameters camera(camera_parameters, distortion, cv::Size(1920,1080));

  //cv::Mat color;
  cv::Mat grey, color;
  while(!to_stop)
  {
    grey = k2g.getGrey();
    //cv::cvtColor(color, grey, CV_BGR2GRAY);

    //MDetector.detect(grey, markers, camera, 0.045);
    MDetector.detect(grey, markers);

    elapsed = elapsed + float(clock() - begin_time) / CLOCKS_PER_SEC;

    if(markers.size() > 0)
      for(int i = 0; i < markers.size(); ++i)
      {
        // Get marker position
        markers[i].draw(grey, cv::Scalar(0, 0, 255), 2);
        aruco::CvDrawingUtils::draw3dCube(grey, markers[i], camera);

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

    if(visualization){
      cv::imshow("hands", grey);
      int c = cv::waitKey(1);
      if((char)c == 'q' )
      {
        to_stop = true;
        std::cout << "Stop requested by hand detector" << std::endl;
      }
    }
  }


  //Destroy the window
  cvDestroyWindow("hands");
  k2g.shutDown();
  return;
}

    