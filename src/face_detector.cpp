#include "face_detector.h"

FaceDetector::FaceDetector(int session): session_(session), capture(0)
{
	cv::gpu::printShortCudaDeviceInfo(cv::gpu::getDevice());

	if( !face_cascade_.load( face_cascade_name_ ) )
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_ << std::endl; 
		to_stop_ = true;
	}
	if( !cascade_gpu_.load( face_cascade_name_ ) )
	{ 
		std::cout << "--(!)Error loading " << face_cascade_name_ << std::endl; 
		to_stop_ = true;
	}
	 cv::namedWindow("face");

}

int FaceDetector::detect(KinectManagerExchange & kme, DataWriter & websocket)
{
	while(!to_stop_)
	{	/*
		if(!kme.getColorGRAY(color_))
	    {
	      std::cout << "failed to fetch data from the kinect\n";
	      return -1;
	    }*/

        capture >> color_;

		cvtColor(color_,tmp,CV_BGR2GRAY); 
		detectAndDisplayGPU( tmp );


		int c = cv::waitKey(10);
		if( (char)c == 'q' ) {
			to_stop_ = true;
		}
	}
	return 0;	
}

void FaceDetector::detectAndDisplayGPU(cv:: Mat image)
{
	std::vector<cv::Rect> res;
   
    int detections_num;
    cv::Mat faces_downloaded;
    cv::gpu::GpuMat facesBuf_gpu;
    cv::Mat im(image.size(), CV_8UC1);
  
    image.copyTo(im);
    
    cv::gpu::GpuMat gray_gpu(im);

    cascade_gpu_.visualizeInPlace = false;
    cascade_gpu_.findLargestObject = findLargestObject_;
    detections_num = cascade_gpu_.detectMultiScale(gray_gpu, facesBuf_gpu, 1.2,(filterRects_ || findLargestObject_) ? 4 : 0, cv::Size(image.cols/4,image.rows/4));

    facesBuf_gpu.colRange(0, detections_num).download(faces_downloaded);
    cv::Rect *faces = faces_downloaded.ptr<cv::Rect>();

    std::cout << "found " << detections_num << " faces " << std::endl;

    for( size_t i = 0; i < detections_num; ++i )
  	{
	    cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
	    cv::ellipse( image, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

  	}
	//-- Show what you got
	cv::imshow( "face", image );

    gray_gpu.release();
    facesBuf_gpu.release();

}

void FaceDetector::detectAndDisplay( cv::Mat frame )
{
  std::vector<cv::Rect> faces;
  cv::Mat frame_gray;

  cv:cvtColor( frame, frame_gray, CV_BGR2GRAY );
  cv::equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  face_cascade_.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

  for( size_t i = 0; i < faces.size(); ++i )
  {
    cv::Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
    cv::ellipse( frame, center, cv::Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar( 255, 0, 255 ), 4, 8, 0 );

  }
  //-- Show what you got
  cv::imshow( "face", frame );
 }