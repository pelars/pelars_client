#pragma once
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include "tinyxml2.h"
#include <fstream>
#include <chrono>
#include "termination.h"
#ifdef HAS_QRENCODE
#include <qrencode.h>
#endif

extern bool to_stop;
extern bool online;

class SessionManager {

public:

	SessionManager(std::string endpoint, const bool test = false);
	int getNewSession(double time = 0);
	void closeSession(int session, double time = 0);
	void createUser();
	void login();
	std::string getToken();

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
	bool error_, test_;

};