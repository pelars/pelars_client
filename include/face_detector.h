#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <data_writer.h>
#include <boost/network/protocol/http/client.hpp>
#include <boost/filesystem.hpp>
#include <json/json.h>
#include <vector>
#include "opt.h"
#include "gstreamer_grabber.h"
#include "screen_grabber.h"
#include "image_sender.h"
#include <base64.h>


extern bool to_stop;
extern bool visualization;
extern double interval;
extern bool snapshot_people;
extern bool snapshot_screen;

void detectFaces(DataWriter & websocket, ScreenGrabber & screen_grabber, ImageSender & image_sender_screen, ImageSender & image_sender_people, const int face_camera_id);
