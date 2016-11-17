#pragma once
#include <signal.h>
#include <iostream>
#include <condition_variable>
#include <vector>
#include <memory>
#include "trigger.h"
#include "image_frame.h"
#include "pooledchannel.hpp"

extern bool to_stop;

// Triggers
extern std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>> pc_trigger;

// Webcam frames message channels
extern std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> pc_webcam;

// Kinect frames message channels
extern std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> pc_kinect;

// Screen frames message channel
extern std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> pc_screen;

void sig_handler(int signum);
void terminateMe();
