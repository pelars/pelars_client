#pragma once
#include <boost/asio.hpp>
#include "data_writer.h"
#include <json/json.h>
#include <chrono>
#include <qrencode.h>
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>
#include <string>

// Asion communication service and Asio keep alive
extern boost::asio::io_service io;
extern bool online;
extern bool to_stop;

void aliver(const boost::system::error_code& /*e*/);

void asiothreadfx();

// Calculate a time interval
double deltats(const struct timespec & a, const struct timespec & b);

const std::string currentDateTime();

struct MiniEncapsule{

	MiniEncapsule(DataWriter & websocket, int session): websocket_(websocket), session_(session){
	}

	void parse(std::string message){
		bool parsedSuccess = reader.parse(message, root_, false);
		root_["obj"]["session"] = session_;
		out_message_ = writer_.write(root_);

	}

	DataWriter & websocket_;
	int session_;
	std::string out_message_;
	Json::Value root_;
	Json::StyledWriter writer_;
	Json::Reader reader;

};

class TimedSender
{   

public:
	TimedSender(double step): step_(step) {
		inited_ = false;
	}

	bool needSend()
	{
		if(!inited_)
		{
			inited_ = true;
			last_ = std::chrono::system_clock::now();
			return true;
		}
		else if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_).count() >= step_)
		{
			last_ = std::chrono::system_clock::now();
			return true;
		}
		else
			return false;
	}

private:
	double step_;
	bool inited_;
	std::chrono::time_point<std::chrono::system_clock> last_;

};

void createQrImage(cv::Mat & qr, QRcode * code);

void drawQr(int session);