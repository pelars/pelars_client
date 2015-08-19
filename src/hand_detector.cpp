#include "hand_detector.h"

template <typename T> string toString(T t)
{
    ostringstream out;
    out << t;
    return out.str();
}

void create_pipeline(gstData *data)
{
    data->pipeline = gst_pipeline_new ("pipeline");
    gst_element_set_state (data->pipeline, GST_STATE_NULL);
}

/// gst-launch-1.0 -v -e v4l2src device=/dev/video0 ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! \ h264parse ! avdec_h264 ! xvimagesink sync=false

gboolean init_video_capture(gstData *data)
{    
	// GST_PLUGIN_SYSTEM_PATH
    data->video_source = gst_element_factory_make("v4l2src", "video_source_live");
    data->vsource_capsfilter = gst_element_factory_make ("capsfilter", "vsource_cptr_capsfilter");
    data->parser = gst_element_factory_make("h264parse", "parser");
    data->decoder = gst_element_factory_make("avdec_h264", "decoder");
    data->colorSpace1 = gst_element_factory_make("videoconvert", "csp1");        
    data->colorSpace2 = gst_element_factory_make("videoconvert", "csp2");        
    data->cspappsink_capsfilter = gst_element_factory_make ("capsfilter", "cspappsink_capsfilter");

    data->appsink = gst_element_factory_make("appsink", "asink");
    gst_app_sink_set_emit_signals((GstAppSink*)data->appsink, true);
	gst_app_sink_set_drop((GstAppSink*)data->appsink, true);
	gst_app_sink_set_max_buffers((GstAppSink*)data->appsink, 1);


     if (!data->video_source || !data->appsink || !data->vsource_capsfilter ||!data->parser || !data->decoder || !data->cspappsink_capsfilter)
    {
        g_printerr ("Not all elements for video were created: vs:%p vsf:%p app:%p parser:%p decoder:%p\n",data->video_source,data->vsource_capsfilter ,data->appsink,data->parser,data->decoder);
        return FALSE;
    }
     
    g_signal_connect( data->pipeline, "deep-notify", G_CALLBACK( gst_object_default_deep_notify ), NULL );        
     
    gst_app_sink_set_emit_signals((GstAppSink*)data->appsink, true);
    gst_app_sink_set_drop((GstAppSink*)data->appsink, true);
    gst_app_sink_set_max_buffers((GstAppSink*)data->appsink, 1);    
     
    // video/x-h264,width=1920,height=1080,framerate=30/1
    data->srcdeinterlace_caps = gst_caps_from_string("video/x-h264, width=(int)1920, height=(int)1080, fframerate=30/1");        
    if (!data->srcdeinterlace_caps)
        g_printerr("1. Could not create media format string.\n");        
    g_object_set (G_OBJECT (data->vsource_capsfilter), "caps", data->srcdeinterlace_caps, NULL);
    gst_caps_unref(data->srcdeinterlace_caps);        
     
     
//    data->cspappsink_caps = gst_caps_from_string("video/x-raw, format=(string)BGR, width=(int)1920, height=(int)1080, framerate=(fraction)30/1");        
    data->cspappsink_caps = gst_caps_from_string("video/x-raw, format=(string)GRAY8, width=(int)1920, height=(int)1080, framerate=(fraction)30/1");        
    if (!data->cspappsink_caps)
        g_printerr("3. Could not create media format string.\n");        
   g_object_set (G_OBJECT (data->cspappsink_capsfilter), "caps", data->cspappsink_caps, NULL);    
    gst_caps_unref(data->cspappsink_caps);        
            
     

    data->bin_capture = gst_bin_new ("bin_capture");        
     
    gst_bin_add_many (GST_BIN (data->bin_capture), data->video_source, data->vsource_capsfilter,data->parser,data->colorSpace1,data->cspappsink_capsfilter,data->colorSpace2,data->decoder, data->appsink, NULL);
     
     // WE RELY ON THE FACT THAT THE x264 decoder uses YUV420 so we can grab the first HxWx8bit data as grayscale
    if (gst_element_link_many(data->video_source, data->vsource_capsfilter,data->parser,data->decoder, /*data->colorSpace1,data->cspappsink_capsfilter,data->colorSpace2,*/ data->appsink, NULL) != TRUE)
    {
        g_printerr ("video_src linking failed.\n");
        return FALSE;
    }        
     
    cout << "Returns from init_video_capture." << endl;
    return TRUE;
}
 
void delete_pipeline(gstData *data)
{
    g_print ("delete_pipeline\n");
    gst_element_set_state (data->pipeline, GST_STATE_NULL);
    g_print ("Pipeline set to NULL\n");
  //  gst_object_unref (data->bus);
    gst_object_unref (data->pipeline);
    g_print ("Pipeline deleted\n");
}
 
gboolean add_bin_capture_to_pipe(gstData *data)
{
    if((gst_bin_add(GST_BIN (data->pipeline), data->bin_capture)) != TRUE)
    {
        g_print("bin_capture not added to pipeline\n");
    }
     
    if(gst_element_set_state (data->pipeline, GST_STATE_NULL) == GST_STATE_CHANGE_SUCCESS)
    {        
        return TRUE;
    }
    else
    {
        cout << "Failed to set pipeline state to NULL." << endl;
        return FALSE;        
    }
}
 
gboolean remove_bin_capture_from_pipe(gstData *data)
{
    gst_element_set_state (data->pipeline, GST_STATE_NULL);
    gst_element_set_state (data->bin_capture, GST_STATE_NULL);
    if((gst_bin_remove(GST_BIN (data->pipeline), data->bin_capture)) != TRUE)
    {
        g_print("bin_capture not removed from pipeline\n");
    }    
    return TRUE;
}
 
gboolean start_capture_pipe(gstData *data)
{
	int r = 0;
    if((r = gst_element_set_state (data->pipeline, GST_STATE_PLAYING)) != GST_STATE_CHANGE_FAILURE)
    	return TRUE;
    else
    {
        cout << "Failed to set pipeline state to PLAYING " << r << endl;
        return FALSE;
    }
}
 
gboolean stop_capture_pipe(gstData *data)
{
    gst_element_set_state (data->bin_capture, GST_STATE_NULL);
    gst_element_set_state (data->pipeline, GST_STATE_NULL);
    return TRUE;
}
 
gboolean deinit_video_live(gstData *data)
{
    gst_element_set_state (data->pipeline, GST_STATE_NULL);
    gst_element_set_state (data->bin_capture, GST_STATE_NULL);
//    gst_object_unref (data->bin_capture);
    return TRUE;
}
 
gboolean check_bus_cb(gstData *data)
{
    GError *err = NULL;                
    gchar *dbg = NULL;   
           
    g_print("Got message: %s\n", GST_MESSAGE_TYPE_NAME(data->msg));
    switch(GST_MESSAGE_TYPE (data->msg))
    {
        case GST_MESSAGE_EOS:       
            g_print ("END OF STREAM... \n");
            break;
 
        case GST_MESSAGE_ERROR:
            gst_message_parse_error (data->msg, &err, &dbg);
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
            g_printerr ("Unexpected message of type %d", GST_MESSAGE_TYPE (data->msg));
            break;
    }
    return TRUE;
}
 
void get_pipeline_bus(gstData *data)
{
    data->bus = gst_element_get_bus (data->pipeline);
//    data->msg = gst_bus_poll (data->bus, GST_MESSAGE_EOS | GST_MESSAGE_ERROR, -1);
    if(GST_MESSAGE_TYPE (data->msg))
    {
        check_bus_cb(data);
    }
    gst_message_unref (data->msg);
}

void xx(const char * p)
{
	std::cout << p << std::flush;
}


void handDetector(KinectManagerExchange & kme, DataWriter & websocket, int session){

	// OpenCV matrixes to contain the data acquired from the kinect
	cv::Mat depth;

	aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    sleep(2); //else opencv crash for parallel windows instantiation
    cv::namedWindow("hands");

    Json::Value root;
    Json::StyledWriter writer;
    float x, y, z;
    bool kinect = false;
  //  capture.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
  //   capture.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

    IplImage *frame = NULL;    
			
    gstData gstreamerData;
    GstBuffer *gstImageBuffer;

    gst_init(NULL, NULL);

    create_pipeline(&gstreamerData);
    xx("pipeline created\n");
    if(init_video_capture(&gstreamerData))
    {        
    xx("init video capture done\n");
        add_bin_capture_to_pipe(&gstreamerData);    
    xx("app bin capture to pipe \n");
    if(!start_capture_pipe(&gstreamerData))
    	return;
    xx("start\n");
     
        cout << "Starting while loop..." << endl;

   

        int skip = 0;
	while(!to_stop){
		   //	get_pipeline_bus(&gstreamerData);    

               GstSample * sample = gst_app_sink_pull_sample((GstAppSink*)gstreamerData.appsink);

               if(!sample)
               	{
               		skip++;
               	continue;
               }
					GstBuffer*  gstImageBuffer=	gst_sample_get_buffer (sample);
			  if(!gstImageBuffer)
			  	{
			  		   //         gst_sample_unref(sample);
			  		            continue;
			  	}
         
            if (!frame)
            {        
                frame = cvCreateImage(cvSize(1920, 1080), IPL_DEPTH_8U, 1);
                cout << "frame created" << endl;
                     	
                if (frame == NULL)
                {
                    g_printerr("IplImageFrame is null.\n");
                    exit(0);
                }
                else
                	std::cout << "image size is " << frame->imageSize << " buffer size is " << gst_buffer_get_size(gstImageBuffer) << " planaer " << 	gst_buffer_n_memory(gstImageBuffer) << std::endl;
            }
            GstCaps * c = gst_sample_get_caps(sample);
            GST_WARNING ("caps are %" GST_PTR_FORMAT, c);
            if(skip)
            std::cout << "skipped " << skip << std::endl;

//  
			gst_buffer_extract(gstImageBuffer,0,frame->imageData,frame->imageSize);
			cv::Mat color(frame); 

			 MDetector.detect(color, markers);
	    	if(markers.size() > 0){
   			for(int i = 0; i < markers.size(); ++i)
   				if(markers[i].id != 1023){
   				// Get marker position
				markers[i].draw(color, cv::Scalar(0, 0, 255), 2);
				x = markers[i][0].x;
				y = markers[i][0].y;

				kinect ? z = depth.at<short>((int)y, (int)x) : 0.0f;

				root["obj"]["type"] = "hand";
		        root["obj"]["id"] = markers[i].id;
		        root["obj"]["x"] = x;
		        root["obj"]["y"] = y;
		        root["obj"]["z"] = z/1000;
		        root["obj"]["time"] = deltats(orwl_gettime(), start);
		        std::string out_string = writer.write(root);
		        //std::cout << "sending " << out_string << std::endl;
		        // Send the message online and store it offline
		        if(online)
		          io.post( [&websocket, out_string]() {
		               websocket.writeData(out_string);
		           });

		        websocket.writeLocal(out_string);
			
   			}	
    	}


	//		gst_buffer_unref(gstImageBuffer);
      	//      gst_sample_unref(sample);
		//	cvShowImage("hands", color);  
		cv::imshow("hands", color);
			int cc = cvWaitKey(1);                    
			if(cc != -1)
				break;
			               skip = 0;

    /*
            if (frame)
            {
                cvShowImage("Toradex Face Detection Demo with Gstreamer", frame);
            }
            gst_buffer_unref(buffer);
            buffer = NULL;            
            pthread_mutex_unlock(&threadMutex);    
            cvWaitKey(1);*/                
		
	}
}
	 else
    {
        exit(1);
    }

        //Destroy the window
    cvDestroyWindow("hands");
    deinit_video_live(&gstreamerData);    
    remove_bin_capture_from_pipe(&gstreamerData);
    delete_pipeline(&gstreamerData);
     
    return;

}

    