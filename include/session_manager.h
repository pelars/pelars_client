#pragma once
#include <boost/network/protocol/http/client.hpp>
#include <string>
#include <json/json.h>
#include "tinyxml2.h"


extern bool to_stop;
extern bool online;

class SessionManager {

public:

	SessionManager(std::string endpoint);

	int getNewSession();

	void closeSession(int session);

	void createUser();

private:

	int session_;
	boost::network::http::client client_;
	boost::network::http::client::response response_;
	std::string endpoint_, session_manager_response_;
	std::stringstream string_stream_;
	tinyxml2::XMLDocument data_;
};