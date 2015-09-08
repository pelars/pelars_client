#include <gstreamer_grabber2.h>

GstreamerGrabber2::GstreamerGrabber2(const char * device, const int width, const int height, const bool yuv420, const bool testsrc, const char * command):
	width_(width), height_(height){
	gst_init(NULL,NULL);
   	g_object_set (src_, "device", device, NULL);
   	const char * outformat = yuv420 ? "I420" : "RGB";
   	if(!(command == ""))
   		strcpy(buffer_, command);
	else
	   	if(!testsrc)
			sprintf(buffer_,"v4l2src name=src ! queue ! video/x-h264,width=%d,height=%d,framerate=30/1 ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=%s, width=%d,height=%d ! appsink name=sink",
		                    width_, height_, outformat, width_, height_);
		else
			sprintf(buffer_,"videotestsrc ! video/x-raw,format=%s, width=%d,height=%d ! appsink name=sink", outformat, width_, height_);
	pipeline_ = gst_parse_launch(buffer_, &error_);
	if (!pipeline_)
	{
    	g_print ("Parse error: %s\n", error_->message);
    	exit (1);
  	}
  	// extract
	sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
	if(src_)
	{
		src_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
		gst_object_unref(src_);
	}
	// appsink
	appsink_ = (GstAppSink*)(sink_); // TODO cast
	g_signal_connect(pipeline_, "deep-notify", G_CALLBACK(gst_object_default_deep_notify ), NULL);             
	gst_app_sink_set_emit_signals(appsink_, true);
	gst_app_sink_set_drop(appsink_, true);
	gst_app_sink_set_max_buffers(appsink_, 1);    
	gst_element_set_state(pipeline_, GST_STATE_PLAYING);
	bus_ = gst_element_get_bus(pipeline_);
}

GstreamerGrabber2::~GstreamerGrabber2(){
	gst_element_set_state(pipeline_, GST_STATE_NULL);
	gst_object_unref(sink_);
	gst_object_unref(pipeline_);
	gst_object_unref(bus_);
}

 
void GstreamerGrabber2::operator >>(cv::Mat & frame){

    GstSample * sample = gst_app_sink_pull_sample(appsink_);
    GstBuffer * gstImageBuffer= gst_sample_get_buffer(sample); 
    GstCaps * c = gst_sample_get_caps(sample); 
    GST_WARNING ("caps are %" GST_PTR_FORMAT, c);
    frame.resize(width_, height_);

    gst_buffer_extract(gstImageBuffer, 0, frame.data, gsize());
    gst_buffer_unref(gstImageBuffer);
}
 
void GstreamerGrabber2::capture(IplImage * frame)
{
	GstSample * sample = gst_app_sink_pull_sample(appsink_);
	GstBuffer * gstImageBuffer=	gst_sample_get_buffer(sample);	
    GstCaps * c = gst_sample_get_caps(sample);
    GST_WARNING ("caps are %" GST_PTR_FORMAT, c);

	gst_buffer_extract(gstImageBuffer, 0, frame->imageData, frame->imageSize);
	gst_buffer_unref(gstImageBuffer);
}