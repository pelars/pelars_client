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

// Calculate a time interval
double deltats(const struct timespec & a, const struct timespec & b)
{
  return a.tv_sec-b.tv_sec + ((a.tv_nsec - b.tv_nsec) / 1000000000.0);
}

const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

void drawQr(int width, int repetitions, int session)
{
	QRcode * qrcode;
	cv::namedWindow("qr");
	cv::moveWindow("qr", 10, 10);
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
		return;
	}
	for(int y = 0; y < QR_width; y++)
  		for(int x = 0; x < QR_width; x++)
			for(int yi = 0; yi < repetitions; yi++)
				for(int xi = 0; xi < repetitions; xi++)
	  				qr.at<char>(y * repetitions + yi + position, x * repetitions + xi + position) = qrcode->data[y * qrcode->width + x] & 1 ? 0 : 255;
	  		
  	cv::imshow("qr", qr);
	int c = cv::waitKey(1);
}


void printHelp(){

	std::cout << "Usage: ./sensormanager {OPTIONS}" << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "\t-f to track faces" << std::endl;
	std::cout << "\t-h to track hands" << std::endl;
	std::cout << "\t-a to track audio level" << std::endl;
	std::cout << "\t-p to track partile.io sensors" << std::endl;
	std::cout << "\t-q to display a qr code of the session id" << std::endl;
	std::cout << "\t--o=\"path to template file list\" to track objects" << std::endl;
	std::cout << "\t-v to enable visualization" << std::endl;
	std::cout << "\t-i to track the Arduino Ide" << std::endl;
	std::cout << "\t--m=\"float value\" of the marker size" << std::endl;
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