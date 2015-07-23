#include "face_detector.h"


void detectFaces(DataWriter & websocket, int session)
{
	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	std::string face_cascade_name_ = "../../data/haarcascade_frontalface_alt.xml";
	std::string face_cascade_name_gpu_ = "../../data/haarcascade_frontalface_alt2.xml";
	cv::CascadeClassifier face_cascade_;
	cv::Mat color, tmp;

	double elapsed;

	cv::gpu::CascadeClassifier_GPU cascade_gpu_;
	bool findLargestObject_ = false;
    bool filterRects_ = true;
    cv::VideoCapture capture(0);

	bool to_stop_ = false;

	if( !cascade_gpu_.load( face_cascade_name_gpu_ ) )
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_gpu_ << std::endl; 
		to_stop_ = true;
	}
	cv::namedWindow("face");

	while(!to_stop_)
	{	

	    capture >> color;

		cvtColor(color, tmp, CV_BGR2GRAY); 

		std::vector<cv::Rect> res;
	   
	    int detections_num;
	    cv::Mat faces_downloaded;
	    cv::gpu::GpuMat facesBuf_gpu;
	    cv::Mat im(tmp.size(), CV_8UC1);
	  
	    tmp.copyTo(im);
	    
	    cv::gpu::GpuMat gray_gpu(im);

	    cascade_gpu_.visualizeInPlace = false;
	    cascade_gpu_.findLargestObject = findLargestObject_;
	    detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, cv::Size(tmp.cols,tmp.rows), cv::Size(), 1.05, (filterRects_ || findLargestObject_) ? 4 : 0
	    											);
	    //detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, 1.2,(filterRects_ || findLargestObject_) ? 4 : 0, cv::Size(tmp.cols/4,tmp.rows/4));


	    facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
	    cv::Rect *faces = faces_downloaded.ptr<cv::Rect>();

	    //std::cout << "found " << detections_num << " faces " << std::endl;

	    for( int i = 0; i < detections_num; ++i )
	  	{
		    cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		    cv::ellipse( color, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

		     // Preapare JSON message to send to the Collector
	        Json::Value root;
	        Json::Value pos;
	        Json::StyledWriter writer;

	        //Elapsed time from process start
	        elapsed = deltats(orwl_gettime(), start);

	        // Json message
	        root["obj"]["type"] = "face";
	        root["obj"]["session"] = session;
	        root["obj"]["id"] = i;
	        root["obj"]["x"] = faces[i].x; 
	        root["obj"]["x1"] = faces[i].x + faces[i].width;
	        root["obj"]["y"] = faces[i].y;
	        root["obj"]["y1"] = faces[i].y + faces[i].height;
	        root["obj"]["time"] = elapsed; // session relative

	        //Send message
	        std::string out_string = writer.write(root);
	        //std::cout << "sending " << out_string << std::endl;
	        if(online)
          		io.post( [&websocket, out_string]() {
                websocket.writeData(out_string);
                });
            
            websocket.writeLocal(out_string);    
	  	}

		//-- Show what you got
		cv::imshow( "face", color );

	    gray_gpu.release();
	    facesBuf_gpu.release();

	    int c = cv::waitKey(1);
		if( (char)c == 'q' ) {
			to_stop_ = true;
		}
	}
	
	to_stop = true;
}


void detectFacesCPU(DataWriter & websocket, int session)
{

	std::string face_cascade_name = "../../data/lbpcascade_frontalcatface.xml";
	cv::CascadeClassifier face_cascade;
	cv::Mat color, tmp;

	double elapsed;

    cv::VideoCapture capture(0);

	bool to_stop_ = false;

	if( !face_cascade.load( face_cascade_name ) )
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name << std::endl; 
		to_stop_ = true;
	}

	bool findLargestObject_ = false;
    bool filterRects_ = true;

    face_cascade.load( face_cascade_name);
	
	cv::namedWindow("face");

	while(!to_stop_)
	{	

	    capture >> color;

		cvtColor(color, tmp, CV_BGR2GRAY); 
		equalizeHist( tmp, tmp );

		std::vector<cv::Rect> faces;
	   	face_cascade.detectMultiScale( tmp, faces, 1.05, 5, 0,  cv::Size(tmp.cols/5,tmp.rows/5), cv::Size(tmp.cols/3,tmp.rows/3) );


	    for( int i = 0; i < faces.size(); ++i )
	  	{
		    cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
		    cv::ellipse( color, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

		     // Preapare JSON message to send to the Collector
	        Json::Value root;
	        Json::Value pos;
	        Json::StyledWriter writer;

	        //Elapsed time from process start
	        elapsed = deltats(orwl_gettime(), start);

	        // Json message
	        root["obj"]["type"] = "face";
	        root["obj"]["session"] = session;
	        root["obj"]["id"] = i;
	        root["obj"]["x"] = faces[i].x; 
	        root["obj"]["x1"] = faces[i].x + faces[i].width;
	        root["obj"]["y"] = faces[i].y;
	        root["obj"]["y1"] = faces[i].y + faces[i].height;
	        root["obj"]["time"] = elapsed; // session relative

	        //Send message
	        std::string out_string = writer.write(root);
	        //std::cout << "sending " << out_string << std::endl;
	        if(online)
          		io.post( [&websocket, out_string]() {
                websocket.writeData(out_string);
                });
            
            websocket.writeLocal(out_string);    
	  	}

		//-- Show what you got
		cv::imshow( "face", color );

	    int c = cv::waitKey(1);
		if( (char)c == 'q' ) {
			to_stop_ = true;
		}
	}
	
	to_stop = true;
}
