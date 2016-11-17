#include "webcam_publisher.h"

void webcamPublisher(int face_camera_id, const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>>> & pc_webcam){

	synchronizer.lock();

	const unsigned int width = 800;
	const unsigned int height = 448;

	GstreamerGrabber gs_grabber(width, height, face_camera_id);
	IplImage * frame = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

	synchronizer.unlock();
	
	while(!to_stop){

		gs_grabber.capture(frame);

		std::shared_ptr<ImageFrame> image = std::make_shared<ImageFrame>();
		image->color = cv::Mat(frame);
		image->type = std::string("people");

		for(auto & channel : pc_webcam){
			channel->write(image);
		}
	}
	std::cout << "temrinating webcam publisher" << std::endl;

}