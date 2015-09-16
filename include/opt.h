#pragma once
#include <boost/asio.hpp>
#include <iostream>
#include "data_writer.h"
#include <json/json.h>


// Asion communication service and Asio keep alive
extern boost::asio::io_service io;
extern struct timespec start;
extern bool online;

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