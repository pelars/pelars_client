#pragma once
#include <boost/asio.hpp>
#include "data_writer.h"
#include <json/json.h>
#include <chrono>
#ifdef HAS_QRENCODE
#include <qrencode.h>
#endif
#include <opencv2/core/core.hpp>        
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include "session_manager.h"
#include "image_sender.h"
#include <base64.h>
#include <opt_parse.h>

// Asion communication service and Asio keep alive
extern boost::asio::io_service io;
extern bool online;
extern bool to_stop;

void sig_handler(int signum);
void aliver(const boost::system::error_code& /*e*/);
void asiothreadfx();

void printHelp();
void checkEscape(bool visualization, bool special);
int sendCalibration(DataWriter & websocket, bool no_webcam = false, bool no_kinect2 = false);
void drawStatus(Parser & p);

const std::string currentDateTime();

struct MiniEncapsule{

	MiniEncapsule(DataWriter & websocket): websocket_(websocket){
	}

	void parse(std::string message){
		bool parsedSuccess = reader.parse(message, root_, false);
		if(parsedSuccess)
			out_message_ = writer_.write(root_);
	}

	DataWriter & websocket_;
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

