#pragma once
#include <boost/network/protocol/http/client.hpp>
#include <string>
#include <json/json.h>
#include "tinyxml2.h"
#include <fstream>


extern bool to_stop;
extern bool online;

class SessionManager {

public:

	SessionManager(std::string endpoint);

	int getNewSession();

	void closeSession(int session);

	void createUser();

private:

	int session_, user_id_;
	boost::network::http::client client_;
	boost::network::http::client::response response_;
	std::string endpoint_, session_manager_response_;
	tinyxml2::XMLDocument data_;
	Json::Value root_;
	Json::Reader reader_;
	Json::StyledWriter writer_;

};