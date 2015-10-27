#pragma once
#include <boost/program_options.hpp>
#include <boost/any.hpp>
#include <exception>

class FloatException: public exception
{
	virtual const char* what() const throw()
	{
		return "Parsed argument is not a float";
	}
};

class StringException: public exception
{
	virtual const char* what() const throw()
	{
		return "Parsed argument is not a string";
	}
};

class Parser
{

public:

	Parser(int argc, char ** argv):argc_(argc), argv_(argv), description_("Pelars Client Usage")
	{
		description_.add_options()
				("help", "help message")
				("face,f", "track the faces")
				("mongoose,m", "mongoose port for the arduino ide")
				("audio,a", "track audio level")
				("marker,M",  boost::program_options::value<float>(), "marker size")
				("hand,h", "track the hands")
				("particle,p", "track the partile IO sensors")
				("ide,i", "track the Arduino IDE log")
				("visualization,v", "activate visualization")
				("object,O", boost::program_options::value<std::string>(), "object template file")
				("qr,q", "show session as qr code")
				("server,S", boost::program_options::value<std::string>(), "server endpoint")
				("session,x", boost::program_options::value<int>(), "session id")
				("calibration,c", "calibrate cameras")
				("special,s", "special flag for background run")
				("upload,u", boost::program_options::value<std::string>(), "file name to upload")
				("status,j", "displays system status");

		boost::program_options::options_description hidden("Hidden options");
		boost::program_options::store(boost::program_options::command_line_parser(argc_, argv_).options(description_).run(), vm_);
		boost::program_options::notify(vm_);
	}

	bool get(std::string value){
		if(vm_.count(value))
			return true;
		return false;
	}

	void printHelp(){
		std::cout << description_ << std::endl;
	}

	std::string getString(std::string value){
		if(vm_.count(value))
			return vm_[value].as<std::string>();
		throw new StringException;
	}

	float getFloat(std::string value){
		if(vm_.count(value))
			return vm_[value].as<float>();
		throw new FloatException;
	}

	int getInt(std::string value){
		if(vm_.count(value))
			return vm_[value].as<int>();
		throw new std::exception;
	}

private:
	int argc_;
	char ** argv_;
	boost::program_options::variables_map vm_;
	boost::program_options::options_description description_;
};

void drawStatus(Parser & p){

	sleep(2);
	cv::startWindowThread();
	cv::namedWindow("status");

	int offset = 30;
	cv::Mat status(240, 320 , CV_8UC1);
    status.setTo(cv::Scalar(255, 255, 255));
    if(p.get("face")){
    	cv::putText(status, "-face tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("audio")){
    	cv::putText(status, "-audio tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("hand")){
    	cv::putText(status, "-hand tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("particle")){
    	cv::putText(status, "-particle tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
 		offset += 30;
 	}
    if(p.get("qr")){
    	cv::putText(status, "-qr visualization", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("session")){
    	cv::putText(status, "-continuing previous session", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("calibration")){
    	cv::putText(status, "-calibration", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }
    if(p.get("ide")){
    	cv::putText(status, "-arduino tracking", cv::Point(10, offset), 2, 1, cv::Scalar(0, 0, 0), 1.5, 8);
    	offset += 30;
    }

    bool stop = false;
    cv::imshow("status", status);
	while(!to_stop){
		int c = cv::waitKey(1);
		if((char)c == 'q') {
			to_stop = true;
			cv::destroyWindow("status");
		}
	}
}