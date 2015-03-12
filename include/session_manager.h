#pragma once
#include <boost/network/protocol/http/client.hpp>
#include <string>

class SessionManager {

public:

	SessionManager(std::string endpoint);

	int getNewSession();

	void closeSession(int session);

private:

	int session_;
	boost::network::http::client client_;
	boost::network::http::client::response response_;
	std::string endpoint_, type_, teacher_name_, institution_name_, institution_address_, session_endpoint_data_, session_manager_response_;
	std::stringstream string_stream_;

};