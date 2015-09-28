#pragma once
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Infos.hpp>
#include <string.h>
#include <fstream>
#include <vector>
#include <thread>
#include <json/json.h>
#include <boost/algorithm/string.hpp>
#include "data_writer.h"
#include "opt.h"
#include "alttime.h"

extern bool to_stop;
extern bool online;

void sseHandler(DataWriter & websocket);

struct Encapsule{

	Encapsule(DataWriter & websocket, std::string name): websocket_(websocket), name_(name)
	{
		root_["obj"]["type"] = "particle";
		root_["obj"]["name"] = name;
	}

	void addData(std::string data){
		root_["obj"]["data"] = data;
	}
	void prepareData(){
		to_send_ = writer_.write(root_);
	}
	void send(){
		websocket_.writeData(to_send_);
	}
	void localSend(){
		websocket_.writeLocal(to_send_);
	}
	void addTime(){
		root_["obj"]["time"] = deltats(orwl_gettime(), start);
	}

	DataWriter & websocket_;
	std::string name_, body_, to_send_;
	Json::StyledWriter writer_;
	Json::Value root_;

};

class Http{
private:

	CURLM* multi_handle; 
	int handle_count;

public:

	Http();
	~Http();
	void update();
	void addRequest(const char* uri, const std::string token, Encapsule * enc);
	std::vector<CURL*> curl_vector_;

};