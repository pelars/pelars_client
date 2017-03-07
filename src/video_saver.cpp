#include "video_saver.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "timer_manager.h"

void saveVideo(int session,
               std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc,
               bool del, const std::string tname, const int threads) {
  bool inited_color = false;
  bool inited_depth = false;

  std::string video_folder_name = std::string("../../videos");
  std::string video_subfolder_name =
      std::string("../../videos/videos_") + std::to_string(session);

  if (!boost::filesystem::exists(video_subfolder_name)) {
    boost::filesystem::path dir(video_subfolder_name);
    boost::filesystem::create_directories(video_subfolder_name);
  }

  std::ofstream out_time;
  std::ofstream out_oni;
  std::ofstream out_color;
  std::shared_ptr<x264Encoder> x264encoder;
  std::shared_ptr<ImageFrame> frames;

  cv::Mat depth(cv::Size(512, 424), CV_32FC1);
  cv::Mat depth_16(cv::Size(512, 424), CV_16U);
  std::vector<XnUInt8> depth_compressed(512 * 424 * 2);
  XnUInt32 depth_8_size;

  long old_seq = 0;
  long new_seq = 0;
  long lost = 0;
  bool kinect = false;

  TimerManager *tm = TimerManager::instance();
  std::string name_base, name_timestampfile, name_depthfile, name_h264file,
      name_mp4file;

  // read first frames and init variables
  if (!to_stop && pc->read(frames)) {
    unsigned int width = frames->color_.cols;
    unsigned int height = frames->color_.rows;
    bool has4channels = frames->color_.channels() == 4;
    old_seq = frames->seq_number_;
    inited_color = true;
    // 4 channels RGBA is Kinect as PIX_FMT_RGB32
    // 3 channels BGR  is Webcam as PIX_FMT_BGR24

    name_base = video_subfolder_name + "/" + frames->type_ + "_" +
                time2string(std::chrono::high_resolution_clock::now()) + "_" +
                std::to_string(session);
    name_timestampfile = name_base + ".stamp.txt";
    name_depthfile = name_base + ".onidepth";
    name_h264file = name_base + ".h264";
    name_mp4file = name_base + ".mp4";
    name_jsonfile = name_base + ".calib.json";

    out_color.open(name_h264file,std::ios::binary);
	if(!out_color)
	{
		std::cerr << "canont save the rgb file: " << out_color << std::endl;
	}
    x264encoder = std::make_shared<x264Encoder>(out_color);
    x264encoder->initialize(width, height, has4channels ? false : true,
                            has4channels, threads);

    out_time.open(name_timestampfile, std::ios::binary);
	if(!out_time)
	{
		std::cerr << "canont save the timestamp file: " << out_time << std::endl;
	}

    Json::Value root;
    root["type"] = "calibration";
    root["camera"] = "kinect2";
    frame->color_params_.toJSON(root);

    if (frames->hasDepth()) {
      kinect = true;
      inited_depth = true;

      out_oni.open(name_depthfile, std::ios::binary);
      if(!out_oni)
      {
      	std::cerr << "canont save the depth file: " << out_oni << std::endl;
      }
      else
      {
      	  // TODO: decide for different conversion being it 32bit floating point
      	  // https://github.com/jkammerl/compressed_depth_image_transport/blob/master/src/compressed_depth_publisher.cpp
	      frames->depth_.convertTo(depth_16, CV_16U);
	      XnStreamCompressDepth16Z((const XnUInt16 *)depth_16.data,
	                               depth_compressed.size(), depth_compressed.data(),
	                               &depth_8_size);
	      out_oni.write((const char *)&depth_8_size, 4);
	      out_oni.write((const char *)depth_compressed.data(), depth_8_size);
	  }
      frame->depth_params_.toJSON(root["depth"]);
    }

    {
      Json::StyledWriter writer;
      std::ofstream onfj(name_jsonfile, std::ios::binary);
      onfj << writer.write(root);
    }

    x264encoder->encodeFrame((const char *)(frames->color_).data);

    out_time << time2string(frames->time_stamp_) << "\n";
  }

  while (!to_stop && pc->read(frames)) {
    TimerScope ts(tm, tname.c_str());

    new_seq = frames->seq_number_;

    if (new_seq - old_seq > 1 && old_seq != 0) {
      std::cout << "!!!frame lost!!!" << std::endl;
      lost++;
      std::cout << "number " << lost << std::endl;
      if (frames->hasDepth()) {
        std::cout << "from Kinect" << std::endl;
      } else {
        std::cout << "from webcam" << std::endl;
      }
    }

    if (frames->hasDepth()) {
      frames->depth_.convertTo(depth_16, CV_16U);
      XnStreamCompressDepth16Z((const XnUInt16 *)depth_16.data,
                               depth_compressed.size(), depth_compressed.data(),
                               &depth_8_size);
      out_oni.write((const char *)&depth_8_size, 4);
      out_oni.write((const char *)depth_compressed.data(), depth_8_size);
    }
    old_seq = new_seq;

    x264encoder->encodeFrame((const char *)(frames->color_).data);

    out_time << time2string(frames->time_stamp_) << "\n";
  }

  x264encoder->unInitilize();

  out_color.close(); // close before conversion
  x262convertmp4(name_h264file, name_mp4file, del);
}