#include "opt.h"

void aliver(const boost::system::error_code& /*e*/)
{
	
}

void asiothreadfx()
{
	boost::asio::deadline_timer t(io, boost::posix_time::seconds(100000));
	t.async_wait(aliver);
	io.run();
}

const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
	return buf;
}

cv::Mat drawQr(int width, int repetitions, int session)
{
	QRcode * qrcode;
    qrcode = QRcode_encodeString(std::to_string(session).c_str(), 1, QR_ECLEVEL_H, QR_MODE_8, false);

    int scale = qrcode->width * 8;
    int position = scale / 2;

    cv::Mat qr(scale * 2, scale * 2, CV_8UC1);
    qr.setTo(cv::Scalar(255, 255, 255));
    cv::putText(qr, std::to_string(session), cv::Point(10, 50), 6, 1, cv::Scalar(0, 0, 0), 1.5, 8);

	int QR_width = qrcode->width;

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

void checkEscape(bool visualization, bool special){
	if(!visualization && !special){
		while(std::cin.get() != 27){
			if (to_stop){
				break;
			}
		}
		to_stop = true;
		std::cout << "Stopping" << std::endl;
	}
}
