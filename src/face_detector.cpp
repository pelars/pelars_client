#include "face_detector.h"


void detectFaces(DataWriter & websocket)
{
	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	//std::string face_cascade_name_ = "../../data/haarcascade_frontalface_alt.xml";
	std::string face_cascade_name_gpu_ = "../../data/haarcascade_frontalface_alt2.xml";
	cv::CascadeClassifier face_cascade_;
	cv::Mat gray;

	cv::gpu::CascadeClassifier_GPU cascade_gpu_;
	bool findLargestObject_ = false;
    bool filterRects_ = true;

 	//GstreamerGrabber2 gs_grabber2("/dev/video0", 1920, 1080, true, true);
 	GstreamerGrabber gs_grabber(640, 480, 0);
    IplImage * frame = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1); 

	if(!cascade_gpu_.load(face_cascade_name_gpu_))
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_gpu_ << std::endl; 
		to_stop = true;
	}
	
	if(visualization)
		cv::namedWindow("face");

	TimedSender timer(interval);
	bool to_send;

	// Preapare JSON message to send to the Collector
	Json::Value root;
	Json::StyledWriter writer;
	root["obj"]["type"] = "face";

	while(!to_stop)
	{	
	    //capture >> color;
		gs_grabber.capture(frame);
	    cv::Mat gray(frame);

		//cvtColor(color, gray, CV_BGR2GRAY); 

		std::vector<cv::Rect> res;
	   
	    int detections_num;
	    cv::Mat faces_downloaded;
	    cv::gpu::GpuMat facesBuf_gpu;
	    cv::Mat im(gray.size(), CV_8UC1);
	  
	    gray.copyTo(im);
	    
	    cv::gpu::GpuMat gray_gpu(im);

	    cascade_gpu_.visualizeInPlace = false;
	    cascade_gpu_.findLargestObject = findLargestObject_;
	    detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, cv::Size(gray.cols,gray.rows), cv::Size(), 1.05, (filterRects_ || findLargestObject_) ? 4 : 0);

	    facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
	    cv::Rect * faces = faces_downloaded.ptr<cv::Rect>();
	    
	    if(detections_num > 0)
	    	to_send = timer.needSend();
	    for(int i = 0; i < detections_num; ++i)
	  	{
		    cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
		    cv::ellipse(gray, center, cv::Size( faces[i].width * 0.5, faces[i].height * 0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0);

	        if(to_send){

		        // Json message
		        root["obj"]["id"] = i;
		        root["obj"]["x"] = faces[i].x; 
		        root["obj"]["x1"] = faces[i].x + faces[i].width;
		        root["obj"]["y"] = faces[i].y;
		        root["obj"]["y1"] = faces[i].y + faces[i].height;
		        root["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();

		        //Send message
		        std::string out_string = writer.write(root);
		        if(online){
	          		io.post( [&websocket, out_string]() {
	                websocket.writeData(out_string);
	                });
	            }
	            websocket.writeLocal(out_string);  
	        }
	    }

	    gray_gpu.release();
		facesBuf_gpu.release();

		//-- Show what you got
		if(visualization){
			cv::imshow( "face", gray);
			int c = cv::waitKey(1);
			if((char)c == 'q' ) {
				to_stop = true;
				std::cout << "stop requested by face detector" << std::endl;
			}	
		}

		
		
	}	
}

