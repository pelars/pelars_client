#pragma once
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>        
#include <opencv2/features2d/features2d.hpp>
#include "kinect_grabber.h"
#include "data_writer.h"
#include <iostream>
#include "alttime.h"
#include <json/json.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <gst/gst.h> 
#include <gst/app/gstappsink.h>
#include <glib.h>

extern bool to_stop;
extern bool online;

typedef struct _CustomData
{
    GstElement *appsink;
    GstElement *colorSpace1,*colorSpace2;    
    GstElement *pipeline;
    GstElement *vsource_capsfilter, *cspappsink_capsfilter;
    GstElement *bin_capture;
    GstElement *video_source;     
    GstElement *parser;
    GstElement *decoder;
    GstCaps *srcdeinterlace_caps;
    GstCaps*cspappsink_caps;    
    GstBus *bus;
    GstMessage *msg;        
}gstData;

template <typename T> string toString(T t);

void create_pipeline(gstData *data);

gboolean init_video_capture(gstData *data);

void delete_pipeline(gstData *data);

gboolean add_bin_capture_to_pipe(gstData *data);

gboolean remove_bin_capture_from_pipe(gstData *data);

gboolean start_capture_pipe(gstData *data);

gboolean stop_capture_pipe(gstData *data);

gboolean deinit_video_live(gstData *data);

gboolean check_bus_cb(gstData *data);

void get_pipeline_bus(gstData *data);

void xx(const char * p);

void handDetector(KinectManagerExchange & kme, DataWriter & websocket, int session);