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

  //GstreamerGrabber gs_grabber(1920, 1080, 1);
  //GstreamerGrabber2 gs_grabber2("/dev/video0", 1920, 1080, true, false);
  //IplImage * frame = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 1); //TODO 

  //Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("../../data/calibration/rgb_calibration.yaml", "../../data/calibration/depth_calibration.yaml", "../../data/calibration/pose_calibration.yaml");

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
  if(freenect2.enumerateDevices() == 0)
  {
    std::cout << "no kinect2 connected!" << std::endl;
    to_stop = true;
  }
  std::string serial = freenect2.getDefaultDeviceSerialNumber();
  //OPENGL
  pipeline = new libfreenect2::OpenGLPacketPipeline();
  //OPENCL
  //pipeline = new libfreenect2::OpenCLPacketPipeline();
  dev = freenect2.openDevice(serial, pipeline);
  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color);
  libfreenect2::FrameMap frames;
  //libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

  dev->setColorFrameListener(&listener);
  //dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  clock_t begin_time = clock();
  float elapsed = 0.0;
  z = 0.0f;  //TODO fix when we have depth

  //cv::Mat color;
  TURBO_COLOR = false;
  while(!to_stop)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    //gs_grabber >> frame; 
    //cv::Mat color = Kinect2Grabber::CvFrameRgb<pcl::PointXYZRGB>(k2g).data_.clone();
    cv::Mat color;
    if(TURBO_COLOR)
      color = cv::Mat(rgb->height,rgb->width, CV_8UC3, rgb->data);
    else
      color = cv::Mat(rgb->height,rgb->width, CV_8UC1, rgb->data);

    //gs_grabber2.capture(frame);
    //v::Mat color(frame); 

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

    if(visualization){
      cv::imshow("hands", color);
      int c = cv::waitKey(1);
      if((char)c == 'q' )
      {
        to_stop = true;
        std::cout << "Stop requested by hand detector" << std::endl;
      }
    }
    listener.release(frames);
  }
  dev->stop();
  dev->close();

  //Destroy the window
  cvDestroyWindow("hands");
  //k2g.shutDown();
  //sleep(5);
  return;
}

    