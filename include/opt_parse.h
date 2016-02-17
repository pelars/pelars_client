#pragma once
#include <boost/program_options.hpp>
#include <boost/any.hpp>
#include <exception>
#include <iostream>

class Parser
{

public:

	Parser(int argc, char ** argv);

	bool get(std::string value);
	void printHelp();
	std::string getString(std::string value);
	float getFloat(std::string value);
	int getInt(std::string value);

private:
	int argc_;
	char ** argv_;
	boost::program_options::variables_map vm_;
	boost::program_options::options_description description_;
};

