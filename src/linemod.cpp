#include "linemod.h"

extern float similarity;
extern const std::string currentDateTimeNow;

// Draw the Linemod results
void drawResponse(const std::vector<cv::linemod::Template>& templates,
                  int num_modalities, cv::Mat& dst, cv::Point offset, int T, int objid)
{ // Max 5 different colors for modalities
  static const cv::Scalar COLORS[5] = { CV_RGB(0, 0, 255),
                                        CV_RGB(0, 255, 0),
                                        CV_RGB(255, 255, 0),
                                        CV_RGB(255, 140, 0),
                                        CV_RGB(255, 0, 0) };

  for (int m = 0; m < num_modalities; ++m)
  {
    cv::Scalar color = COLORS[objid % 5];
    for (int i = 0; i < (int)templates[m].features.size(); ++i)
    {
      cv::linemod::Feature f = templates[m].features[i];
      cv::Point pt(f.x + offset.x, f.y + offset.y);
      cv::circle(dst, pt, T / 2, color);
    }
  }
}

// Functions to store detector and templates in single XML/YAML file
cv::Ptr<cv::linemod::Detector> readLinemod(const std::string& filename)
{
  cv::Ptr<cv::linemod::Detector> detector = new cv::linemod::Detector;
  cv::FileStorage fs(filename, cv::FileStorage::READ);
  detector->read(fs.root());

  cv::FileNode fn = fs["classes"];
  for (cv::FileNodeIterator i = fn.begin(), iend = fn.end(); i != iend; ++i)
    detector->readClass(*i);

  return detector;
}

int linemodf(std::ifstream & infile, KinectManagerExchange & kme, DataWriter & websocket, int session)
{
  // Some matching parameters for linemod
  short matching_threshold = 85;
  short learning_lower_bound = 90;
  short learning_upper_bound = 95;

  //CREating data for storing the streams
  std::vector<uint8_t> rgb_buffer_compressedx_;
  std::ofstream onfvideojpeg;
  onfvideojpeg.open("out" + currentDateTimeNow + ".mpg",std::ios::binary);


  //Kinect v1 intrinsic parameters
  const double focal_x_depth = 5.9421434211923247e+02;
  const double focal_y_depth = 5.9104053696870778e+02;
  const double center_x_depth = 3.3930780975300314e+02;
  const double center_y_depth = 2.4273913761751615e+02;

  // Initialize an OpenCV window
  cv::namedWindow("color");

  // Initialize LINEMOD data structures
  std::vector<cv::Ptr<cv::linemod::Detector>> detectors;
  std::vector<int> num_modalities_vector;

  // Load all the templates listed in the input file
  std::string data;
  short i = 0;
  float thresh;
  std::vector<float> thresholds;
  while (infile >> data)
  {
    std::cout << "loading " << data << std::endl;
    detectors.push_back(readLinemod(data));
    infile >> thresh;
    thresholds.push_back(thresh);
    std::cout << "with thresh " << thresh << std::endl;
    std::cout <<"Loaded " << data.c_str() <<" with " << detectors[i]->numClasses() << " classes and "<< detectors[i]->numTemplates() << " templates" << std::endl;
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

  // Creating a thread pool to detect templates in parallel
  ThreadPool pool(std::thread::hardware_concurrency());

  // OpenCV matrixes to contain the data acquired from the kinect
  cv::Mat color, depth;

  // Timing variables
  double matching_time;
  struct timespec begin, end;

  // Encoder and data
  EncDec encoder(false, 640, 480, 0, 0);
  cv::Mat large(480, 640 * 2, CV_8UC3);

  // Linemod variables
  std::vector<cv::Mat> sources;
  cv::Mat display;
  double min;
  double max;

  //Main loop. Executes until 'q' is pressed or there is an error with the kinect acquisition.
  while(!to_stop)  {
    // Acquire depth and color images from the kinect and prepare them for linemod
    if(!kme.get(color, depth))
    {
      std::cout << "failed to fetch data from the kinect\n";
      to_stop = true;
    }
/*
    cv::Mat lab_image;
    cv::cvtColor(color, lab_image, CV_BGR2Lab);
    
    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

   // convert back to RGB
   cv::Mat image_clahe;
   cv::cvtColor(lab_image, color, CV_Lab2BGR);
  */
    sources.push_back(color);
    sources.push_back(depth);
    display = color.clone();

    // Get start time
    begin = orwl_gettime();

    // Prepare structures for the parallel template matching
    std::vector<std::vector<result_t> > results(template_num);
    std::vector<std::string> class_ids;

    // Lambda function to match the tamplates
    auto myFunc = [=, &matches_vector, &websocket, &detectors, &class_ids, &results, &thresholds](unsigned int i) {
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

          if(m.similarity >= thresholds[i] )
          {
            // mutex
            //results
            result_t t;
            t.m = m;
            t.end = orwl_gettime();
            results[i].push_back(t);
          }
          //else
            //std::cout << "bad quality for obj: " << i << " with similarity: " << m.similarity << std::endl;
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
        //root["obj"]["session"] = session;
        root["obj"]["id"] = i;
        root["obj"]["x"] = m.x; // (m.x - center_x_depth) * dz / focal_x_depth;
        root["obj"]["y"] = m.y; // (m.y - center_y_depth) * dz / focal_y_depth;
        root["obj"]["z"] = dz;
        root["obj"]["pos"] = pos; // maybe not and one at level of ...
        root["obj"]["time"] = elapsed; // session relative

        //Send message
        std::string out_string = writer.write(root);
        if(online)
          io.post( [&websocket, out_string]() {
               websocket.writeData(out_string);
           });
        
        websocket.writeLocal(out_string);    
      }
    }

    //Total matching time 
    end = orwl_gettime();
    matching_time = (end.tv_sec - begin.tv_sec);
    matching_time += (end.tv_nsec - begin.tv_nsec) / 1000000000.0;
    //std::cout << "total matching time " << matching_time << std::endl;
    
    // Get min and max values of the depth map and convert it to grayscale
    cv::minMaxIdx(depth, &min, &max);
    cv::Mat adjMap;
    depth.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min); 

    // Display color and depth
    cv::imshow("adjMap", adjMap);
    cv::imshow("color", display);

    // Store streams
    const unsigned char * pp = (const unsigned char*)display.data;
    encoder.step(pp,(uint16_t*)depth.data);

    // stich 
    cv::Rect roirgb( cv::Point( 0, 0 ), display.size());
    cv::Mat  destinationROI = large( roirgb);
    display.copyTo( destinationROI );

    cv::Rect roidepth( cv::Point( 640, 0 ), adjMap.size());
    destinationROI = large( roidepth);
    adjMap.copyTo( destinationROI );

    rgb_buffer_compressedx_.clear();
    cv::imencode(".jpg",large,rgb_buffer_compressedx_);
    onfvideojpeg.write((char*)&rgb_buffer_compressedx_[0],rgb_buffer_compressedx_.size());


    // Check if any key is pressed and in case process input
    char key = (char)cvWaitKey(1);
    if( key == 'q' || to_stop ){
      std::cout << "stop requested by object recognition" << std::endl;
      break;
    }
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
}



