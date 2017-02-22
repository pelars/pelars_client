#include <glib.h>
#include <iostream>
#include <sstream>
#include <opencv2/core/core.hpp>   
#include "gstreamer_grabber.h"

#ifdef HAS_GSTREAMER
GstreamerGrabber::GstreamerGrabber(int width, int height, int device_id = 0): height_(height), width_(width), device_id_(device_id)
{

	std::cout << "Opening device /dev/video" << device_id << std::endl;
	gst_init(NULL, NULL);
	createPipeline();
	//std::cout << "Pipeline created" << std::endl;
	if(initVideoCapture())
	{        
	   // std::cout << "Init video capture done" << std::endl;
		addBinCaptureToPipe(); 
	   // std::cout << "App bin capture to pipe" << std::endl;   
		if(!startCapturePipe())
		   return;
		//std::cout << "Start" << std::endl;
	}
}

GstreamerGrabber::~GstreamerGrabber()
{
	std::cout << "Closing gstreamer grabber" << std::endl;
	deinitVideoLive();    
	removeBinCaptureFromPipe();
	deletePipeline();
	std::cout << "gstreamer closed" << std::endl;
}

void GstreamerGrabber::createPipeline()
{
	this->pipeline = gst_pipeline_new ("pipeline");
	gst_element_set_state (this->pipeline, GST_STATE_NULL);
}

/// gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 !  h264parse ! avdec_h264 ! xvimagesink sync=false

gboolean GstreamerGrabber::initVideoCapture()
{    
	// GST_PLUGIN_SYSTEM_PATH
	this->video_source = gst_element_factory_make("v4l2src", "video_source_live");
	this->vsource_capsfilter = gst_element_factory_make ("capsfilter", "vsource_cptr_capsfilter");
	this->parser = gst_element_factory_make("h264parse", "parser");
	this->decoder = gst_element_factory_make("avdec_h264", "decoder");
	this->colorSpace1 = gst_element_factory_make("videoconvert", "csp1");        
	this->colorSpace2 = gst_element_factory_make("videoconvert", "csp2");        
	this->cspappsink_capsfilter = gst_element_factory_make ("capsfilter", "cspappsink_capsfilter");

	this->appsink = gst_element_factory_make("appsink", "asink");
	gst_app_sink_set_emit_signals((GstAppSink*)this->appsink, true);
	gst_app_sink_set_drop((GstAppSink*)this->appsink, true);
	gst_app_sink_set_max_buffers((GstAppSink*)this->appsink, 1);


	if (!this->video_source || !this->appsink || !this->vsource_capsfilter ||!this->parser || !this->decoder || !this->cspappsink_capsfilter)
	{
		g_printerr ("Not all elements for video were created: vs:%p vsf:%p app:%p parser:%p decoder:%p\n",this->video_source,this->vsource_capsfilter ,this->appsink,this->parser,this->decoder);
		return FALSE;
	}
	 
	g_signal_connect( this->pipeline, "deep-notify", G_CALLBACK( gst_object_default_deep_notify ), NULL );        
	 
	gst_app_sink_set_emit_signals((GstAppSink*)this->appsink, true);
	gst_app_sink_set_drop((GstAppSink*)this->appsink, true);
	gst_app_sink_set_max_buffers((GstAppSink*)this->appsink, 1);

	char buffer[4098];     

	sprintf(buffer,"video/x-h264, width=(int)%d, height=(int)%d, fframerate=30/1", width_, height_);
	// video/x-h264,width=1920,height=1080,framerate=30/1
	this->srcdeinterlace_caps = gst_caps_from_string(buffer); 

	if(!this->srcdeinterlace_caps)
		g_printerr("1. Could not create media format string.\n"); 

	g_object_set (G_OBJECT (this->vsource_capsfilter), "caps", this->srcdeinterlace_caps, NULL);
	gst_caps_unref(this->srcdeinterlace_caps);
	sprintf(buffer,"video/x-raw, format=(string)BGR, width=(int)%d, height=(int)%d, framerate=(fraction)30/1", width_, height_);
//    this->cspappsink_caps = gst_caps_from_string("video/x-raw, format=(string)GRAY8, width=(int)1920, height=(int)1080, framerate=(fraction)30/1");        
	this->cspappsink_caps = gst_caps_from_string(buffer);  

	if(!this->cspappsink_caps)
		g_printerr("3. Could not create media format string.\n");   

    g_object_set (G_OBJECT (this->cspappsink_capsfilter), "caps", this->cspappsink_caps, NULL);    
	gst_caps_unref(this->cspappsink_caps);        

	sprintf(buffer,"/dev/video%d", device_id_);
	g_object_set (this->video_source, "device", buffer, NULL);
   // g_object_set (filesink, "location", "/i/do/not/exist", NULL);
		
	this->bin_capture = gst_bin_new ("bin_capture");        
	 
	//gst_bin_add_many (GST_BIN (this->bin_capture), this->video_source, this->vsource_capsfilter,this->parser/*this->colorSpace1,this->cspappsink_capsfilter,this->colorSpace2*/,this->decoder, this->appsink, NULL);
	gst_bin_add_many (GST_BIN (this->bin_capture), this->video_source, this->vsource_capsfilter,this->parser,this->colorSpace1,this->cspappsink_capsfilter,this->colorSpace2,this->decoder, this->appsink, NULL);

	// WE RELY ON THE FACT THAT THE x264 decoder uses YUV420 so we can grab the first HxWx8bit this as grayscale
	//if(gst_element_link_many(this->video_source, this->vsource_capsfilter,this->parser,this->decoder /*this->colorSpace1,this->cspappsink_capsfilter,this->colorSpace2*/, this->appsink, NULL) != TRUE)
	if (gst_element_link_many(this->video_source, this->vsource_capsfilter,this->parser,this->decoder, this->colorSpace1,this->cspappsink_capsfilter,this->colorSpace2, this->appsink, NULL) != TRUE)
	{
		g_printerr ("video_src linking failed.\n");
		return FALSE;
	}        
	 
	//std::cout << "Returns from initVideoCapture." << endl;
	return TRUE;
}
 
void GstreamerGrabber::deletePipeline()
{
	//g_print ("delete_pipeline\n");
	gst_element_set_state (this->pipeline, GST_STATE_NULL);
	//g_print ("Pipeline set to NULL\n");
    //  gst_object_unref (this->bus);
	gst_object_unref (this->pipeline);
	//g_print ("Pipeline deleted\n");
}
 
gboolean GstreamerGrabber::addBinCaptureToPipe()
{
	if((gst_bin_add(GST_BIN (this->pipeline), this->bin_capture)) != TRUE)
	{
		g_print("bin_capture not added to pipeline\n");
	}
	 
	if(gst_element_set_state (this->pipeline, GST_STATE_NULL) == GST_STATE_CHANGE_SUCCESS)
	{        
		return TRUE;
	}
	else
	{
		std::cout << "Failed to set pipeline state to NULL." << std::endl;
		return FALSE;        
	}
}
 
gboolean GstreamerGrabber::removeBinCaptureFromPipe()
{
	gst_element_set_state (this->pipeline, GST_STATE_NULL);
	gst_element_set_state (this->bin_capture, GST_STATE_NULL);
	if((gst_bin_remove(GST_BIN (this->pipeline), this->bin_capture)) != TRUE)
	{
		g_print("bin_capture not removed from pipeline\n");
	}    
	return TRUE;
}
 
gboolean GstreamerGrabber::startCapturePipe()
{
	int r = 0;
	if((r = gst_element_set_state (this->pipeline, GST_STATE_PLAYING)) != GST_STATE_CHANGE_FAILURE)
		return TRUE;
	else
	{
		std::cout << "Failed to set pipeline state to PLAYING " << r << std::endl;
		return FALSE;
	}
}
 
gboolean GstreamerGrabber::stopCapturePipe()
{
	gst_element_set_state (this->bin_capture, GST_STATE_NULL);
	gst_element_set_state (this->pipeline, GST_STATE_NULL);
	return TRUE;
}
 
gboolean GstreamerGrabber::deinitVideoLive()
{
	gst_element_set_state (this->pipeline, GST_STATE_NULL);
	gst_element_set_state (this->bin_capture, GST_STATE_NULL);
//    gst_object_unref (this->bin_capture);
	return TRUE;
}
 
gboolean GstreamerGrabber::checkBusCb()
{
	GError *err = NULL;                
	gchar *dbg = NULL;   
		   
	g_print("Got message: %s\n", GST_MESSAGE_TYPE_NAME(this->msg_));
	switch(GST_MESSAGE_TYPE (this->msg_))
	{
		case GST_MESSAGE_EOS:       
			g_print ("END OF STREAM... \n");
			break;
 
		case GST_MESSAGE_ERROR:
			gst_message_parse_error (this->msg_, &err, &dbg);
			if (err)
			{
				g_printerr ("ERROR: %s\n", err->message);
				g_error_free (err);
			}
			if (dbg)
			{
				g_printerr ("[Debug details: %s]\n", dbg);
				g_free (dbg);
			}
			break;
 
		default:
			g_printerr ("Unexpected message of type %d", GST_MESSAGE_TYPE (this->msg_));
			break;
	}
	return TRUE;
}
 
void GstreamerGrabber::getPipelineBus()
{
	this->bus = gst_element_get_bus (this->pipeline);
//    this->msg = gst_bus_poll (this->bus, GST_MESSAGE_EOS | GST_MESSAGE_ERROR, -1);
	if(GST_MESSAGE_TYPE (this->msg_))
	{
		checkBusCb();
	}
	gst_message_unref (this->msg_);
}

void GstreamerGrabber::capture(IplImage * frame)
{
	GstSample * sample = gst_app_sink_pull_sample((GstAppSink*)this->appsink);
	  
	GstBuffer * gstImageBuffer=	gst_sample_get_buffer(sample);	

	GstCaps * c = gst_sample_get_caps(sample);
	GST_WARNING ("caps are %" GST_PTR_FORMAT, c);

	gst_buffer_extract(gstImageBuffer, 0, frame->imageData, frame->imageSize);
	gst_buffer_unref(gstImageBuffer);
}

void GstreamerGrabber::operator >>(IplImage * frame){

	GstSample * sample = gst_app_sink_pull_sample((GstAppSink*)this->appsink);
	  
	GstBuffer*  gstImageBuffer = gst_sample_get_buffer(sample); 

	GstCaps * c = gst_sample_get_caps(sample);
	GST_WARNING ("caps are %" GST_PTR_FORMAT, c);
	
	gst_buffer_extract(gstImageBuffer,0,frame->imageData,frame->imageSize);
	gst_buffer_unref(gstImageBuffer);
}

void GstreamerGrabber::capture(std::shared_ptr<IplImage> frame)
{
	GstSample * sample = gst_app_sink_pull_sample((GstAppSink*)this->appsink);
	  
	GstBuffer * gstImageBuffer=	gst_sample_get_buffer(sample);	

	GstCaps * c = gst_sample_get_caps(sample);
	GST_WARNING ("caps are %" GST_PTR_FORMAT, c);

	gst_buffer_extract(gstImageBuffer, 0, frame->imageData, frame->imageSize);
	gst_buffer_unref(gstImageBuffer);
}

void GstreamerGrabber::operator >>(std::shared_ptr<IplImage> frame){

	GstSample * sample = gst_app_sink_pull_sample((GstAppSink*)this->appsink);
	  
	GstBuffer*  gstImageBuffer = gst_sample_get_buffer(sample); 

	GstCaps * c = gst_sample_get_caps(sample);
	GST_WARNING ("caps are %" GST_PTR_FORMAT, c);
	
	gst_buffer_extract(gstImageBuffer,0,frame->imageData,frame->imageSize);
	gst_buffer_unref(gstImageBuffer);
}
#endif

