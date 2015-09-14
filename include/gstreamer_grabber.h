#pragma once
#include <gst/gst.h> 
#include <gst/app/gstappsink.h>
#include <glib.h>
#include <string>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>        

class GstreamerGrabber
{
public:

    GstreamerGrabber(int width, int height);

    ~GstreamerGrabber(void);

    void createPipeline();
    gboolean initVideoCapture();
    void deletePipeline();
    gboolean addBinCaptureToPipe();
    gboolean removeBinCaptureFromPipe();
    gboolean startCapturePipe();
    gboolean stopCapturePipe();
    gboolean deinitVideoLive();
    gboolean checkBusCb();
    void getPipelineBus();
    void capture(IplImage * frame);
    void operator >>(IplImage * frame);


private:

    GstElement * appsink;
    GstElement * colorSpace1, * colorSpace2;    
    GstElement * pipeline;
    GstElement * vsource_capsfilter, * cspappsink_capsfilter;
    GstElement * bin_capture;
    GstElement * video_source;     
    GstElement * parser;
    GstElement * decoder;
    GstElement * file_sink;
    GstCaps * srcdeinterlace_caps;
    GstCaps * cspappsink_caps;    
    GstBus * bus;
    GstMessage * msg_;
    int height_, width_;        
};
