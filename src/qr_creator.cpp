#include "qr_creator.h"

void showQr(int session){
	sleep(1);
	cv::startWindowThread();
	cv::namedWindow("qr");
	cv::moveWindow("qr", 10, 10);
	
	cv::Mat qr;	
	if(session != -1){
		qr = drawQr(512, 8, session);
		cv::imshow("qr", qr);
	}
	else
		std::cout << "No Qr code available since there is no session id" << std::endl;

	bool stop = false;

	while(!to_stop){
		int c = cv::waitKey(1);
		if((char)c == 'q') {
			to_stop = true;
			cv::destroyWindow("qr");
		}
	}
}