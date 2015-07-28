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

void sse_handler(DataWriter & websocket);

struct encapsule{

	encapsule(DataWriter & websocket, std::string name): websocket_(websocket), name_(name){
		root_["obj"]["type"] = "particle";
		root_["obj"]["name"] = name;
	}
	void addData(std::string data){
		root_["obj"]["data"] = data;
		to_send_ = writer_.write(root_);
	}
	void send(){
		websocket_.writeData(to_send_);
	}
	void localSend(){
		websocket_.writeLocal(to_send_);
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
	DataWriter & websocket_;

public:

	Http(DataWriter & websocket);
	~Http();
	void update();
	void addRequest(const char* uri, const std::string token, encapsule * enc);
	std::vector<encapsule> enc_vector_;
	std::vector<CURL*> curl_vector_;

};