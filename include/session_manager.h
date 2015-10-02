#pragma once
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include "tinyxml2.h"
#include <fstream>
#include <chrono>
#include <qrencode.h>

extern bool to_stop;
extern bool online;
extern std::chrono::time_point<std::chrono::system_clock> start;

class SessionManager {

public:

	SessionManager(std::string endpoint);
	int getNewSession();
	void closeSession(int session);
	void createUser();
	void login();

private:

	int session_, user_id_;
	std::string token_, mail_, password_;
	boost::network::http::client client_;
	boost::network::http::client::response response_;
	std::string endpoint_, session_manager_response_;
	tinyxml2::XMLDocument data_;
	Json::Value root_;
	Json::Reader reader_;
	Json::StyledWriter writer_;

};