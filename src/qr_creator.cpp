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

cv::Mat drawQr(int width, int repetitions, int session)
{
	// Create qr code 
	QRcode * qrcode;
    qrcode = QRcode_encodeString(std::to_string(session).c_str(), 1, QR_ECLEVEL_H, QR_MODE_8, false);

    // Initialize data
    int scale = qrcode->width * 8;
    int position = scale / 2;

    // Write session in the opencv window
    cv::Mat qr(scale * 2, scale * 2, CV_8UC1);
    qr.setTo(cv::Scalar(255, 255, 255));
    cv::putText(qr, std::to_string(session), cv::Point(10, 50), 6, 1, cv::Scalar(0, 0, 0), 1.5, 8);
	int QR_width = qrcode->width;

	// Create actual qr code from data
	if(QR_width * repetitions > width)
	{
		std::cout << "overflow" << std::endl;
		return cv::Mat();
	}
	for(int y = 0; y < QR_width; ++y)
  		for(int x = 0; x < QR_width; ++x)
			for(int yi = 0; yi < repetitions; ++yi)
				for(int xi = 0; xi < repetitions; ++xi)
	  				qr.at<char>(y * repetitions + yi + position, x * repetitions + xi + position) = qrcode->data[y * qrcode->width + x] & 1 ? 0 : 255;
	return qr;  		
}