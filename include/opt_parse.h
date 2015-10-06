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

	Parser(int argc, char ** argv):argc_(argc), argv_(argv)
	{
		boost::program_options::options_description description("Pelars Client Usage");
		description.add_options()
				("face,f", "track the faces")
				("help", "help message")
				("mongoose,m", "mongoose port for the arduino ide")
				("audio,a", "track audio level")
				("marker",  boost::program_options::value<float>(), "marker size")
				("hand,h", "track the hands")
				("particle,p", "track the partile IO sensors")
				("ide,i", "track the Arduino IDE log")
				("visualization,v", "activate visualization")
				("object,o", boost::program_options::value<std::string>(), "object template file")
				("qr,q", "show session as qr code")
				("Server,S", boost::program_options::value<std::string>(), "server endpoint")
				("calibration,c", "calibrate cameras")
				("special,s", "special flag for background run");

		boost::program_options::options_description hidden("Hidden options");
		boost::program_options::store(boost::program_options::command_line_parser(argc_, argv_).options(description).run(), vm_);
		boost::program_options::notify(vm_);
	}

	bool get(std::string value){
		if(vm_.count(value))
			return true;
		return false;
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

private:
	int argc_;
	char ** argv_;
	boost::program_options::variables_map vm_;
};