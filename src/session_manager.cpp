#include "session_manager.h"

SessionManager::SessionManager(std::string endpoint): endpoint_(endpoint)
{
	srand (time(NULL));
	// Example data (has to be acquireds somehow)
	type_ = "open";
	teacher_name_ = "test_name";
	institution_name_ = "test_name_2";
	institution_address_ = "test_address";
	session_endpoint_data_ ="?type="+type_+"&teacher_name="+teacher_name_+"&institution_name="+institution_name_+"&institution_address="+institution_address_+"&session_id=";

}

int SessionManager::getNewSession()
{
	// Continue sending possible session id's until a good one is found and the session manager gives a positive answer
    // TODO Make some other check on the session manager liveness and on the time spent asking for a session
	do
	{
		session_ = rand();
		std::cout << "asking for session id " << session_ << std::endl;

		boost::network::http::client::request request(endpoint_ + session_endpoint_data_ + std::to_string(session_));
		response_ = client_.get(request);
		string_stream_ << body(response_);
		session_manager_response_ = string_stream_.str();

	}while(!session_manager_response_.compare("open"));

	// Clear the string stream
	string_stream_.str(std::string());
	return session_;
}

void SessionManager::closeSession(int session)
{
	// Close the session and exit
  	// Preaparing data to close the session
	type_ = "close";
	std::cout << "closing session  " << session << std::endl;
 	session_endpoint_data_ = "?type="+type_+"&teacher_name="+teacher_name_+"&institution_name="+institution_name_+"&institution_address="+institution_address_+"&session_id="+std::to_string(session);
 	//Sending data to close the session with the session manager
	boost::network::http::client::request request_(endpoint_ + session_endpoint_data_);
	response_ = client_.get(request_);
	string_stream_ << body(response_);
	std::string client_response_ = string_stream_.str();
	std::cout << "Session manager resonse " << client_response_ << std::endl;
	string_stream_.str(std::string());
}