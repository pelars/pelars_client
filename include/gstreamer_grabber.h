#pragma once
#ifdef HAS_GSTREAMER
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <memory>   

class GstreamerGrabber
{
public:

    //GstreamerGrabber(int width, int height, int device_id = 0);
    GstreamerGrabber(int width, int height, int device_id, bool h264 = true, const char * xpipeline = 0);

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

    void capture(std::shared_ptr<IplImage> frame);
    void operator >>(std::shared_ptr<IplImage> frame);

    bool isValid() const { return appsink != 0; }

private:

    GstAppSink * appsink;
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
    GError * err_;
    int height_, width_, device_id_;        
};
#endif
