#pragma once
#include <string>
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include <chrono>

class ImageSender{
	
public:

	ImageSender(int session, std::string endpoint, std::string token);
	void send(std::string & data, std::string type);
	operator bool() const { return sending_complete_;}

private:
	std::string endpoint_, token_;
    boost::network::http::client client_;
	boost::network::http::client::response response_;
	Json::Value root_;
	Json::StyledWriter writer_;
	std::string out_string_;
	bool sending_complete_;
};