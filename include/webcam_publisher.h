#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>  
#include <vector>
#include "gstreamer_grabber.h"
#include "pooledchannel.hpp"
#include <memory>
#include <mutex>
#include "opt.h"

extern bool to_stop;
extern std::mutex synchronizer;

void webcamPublisher(int face_camera_id, const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> & pc_webcam);