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

void drawQr(int session){
	QRcode * qrcode;
	cv::namedWindow("qr");
	cv::moveWindow("qr", 10, 10);
    qrcode = QRcode_encodeString(std::to_string(session).c_str(), 1, QR_ECLEVEL_H, QR_MODE_8, false);
    int scale = 8;
    int scaled_qr = qrcode->width * scale;
    cv::Mat qr(scaled_qr * 2, scaled_qr * 2, CV_8UC1);
    qr.setTo(cv::Scalar(255, 255, 255));
	for (int y = 0; y < qrcode->width; ++y)
		for(int x = 0; x < qrcode->width; ++x)
			rectangle(qr, cv::Point(y * scale + scaled_qr / 2, x * scale + scaled_qr / 2), cv::Point((y + 1) * scale + scaled_qr / 2, (x + 1) * scale + scaled_qr / 2), qrcode->data[y * qrcode->width + x] & 1 ? 0x00 : 0xFF, CV_FILLED);
	cv::imshow("qr", qr);
	int c = cv::waitKey(1);
}