#pragma once
#include <curlpp/cURLpp.hpp>
#include <boost/algorithm/string.hpp>
#include "opt.h"

extern bool to_stop;
extern bool online;
extern std::chrono::time_point<std::chrono::system_clock> start; 
extern bool snapshot_people;
extern bool snapshot_table;
extern bool snapshot_screen;

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
		root_["obj"]["time"] = (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count();
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