#pragma once
#include <signal.h>
#include <iostream>
#include <condition_variable>
#include <vector>
#include <memory>
#include "trigger.h"
#include "image_frame.h"
#include "channel_wrapper.hpp"

extern bool to_stop;

// Triggers
extern ChannelWrapper<Trigger> pc_trigger;

// Webcam frames message channels
extern ChannelWrapper<ImageFrame> pc_webcam;

// Kinect frames message channels
extern ChannelWrapper<ImageFrame> pc_kinect;

// Screen frames message channel
extern ChannelWrapper<ImageFrame> pc_screen;

void sig_handler(int signum);
void terminateMe();
