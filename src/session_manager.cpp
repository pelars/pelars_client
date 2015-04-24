#include "session_manager.h"

SessionManager::SessionManager(std::string endpoint): endpoint_(endpoint)
{
	srand (time(NULL));
	// Example data (has to be acquireds somehow)
	type_ = "open";
	teacher_name_ = "test_name";
	institution_name_ = "test_name_2";
	institution_address_ = "test_address";
	//session_endpoint_data_ ="?type="+type_+"&teacher_name="+teacher_name_+"&institution_name="+institution_name_+"&institution_address="+institution_address_+"&session_id=";
	
}

int SessionManager::getNewSession()
{
	// Continue sending possible session id's until a good one is found and the session manager gives a positive answer
    // TODO Make some other check on the session manager liveness and on the time spent asking for a session
    bool error = false;
    Json::Value root;
    Json::StyledWriter writer;

    // Json message
    root["teacher_name"] = "test";
    root["institution_name"] = "institution_test";
    root["institution_address"] = "institution_test_address";
    std::string out_string = writer.write(root);

	do
	{
		session_ = rand();
		std::cout << "Requesting session id " << session_ << std::endl;
		try{
			boost::network::http::client::request endpoint(endpoint_ + std::to_string(session_));
			//response_ = client_.get(endpoint); 
			response_ = client_.put(endpoint, out_string);
			string_stream_ << body(response_);
			session_manager_response_ = string_stream_.str();
			std::cout << "got response " << session_manager_response_  <<std::endl;
			session_manager_response_.erase(std::remove(session_manager_response_.begin(), session_manager_response_.end(), '\n'), session_manager_response_.end());
			string_stream_.str(std::string());
		}
		catch (std::exception& e){
			std::cout << "Error during the session request: " << e.what()  << "; setting session id to 0" <<std::endl;
			error = true;
			online = false;
			session_ = -1;
			break;
		}
		
	}while(session_manager_response_.find("open") == std::string::npos);
	if (!error)
		std::cout << "Got session id " << session_ << std::endl;
	return session_;
}


void SessionManager::closeSession(int session)
{
	// Close the session and exit
  	// Preaparing data to close the session

  	Json::Value root;
    Json::StyledWriter writer;

    // Json message
    root["op_code"] = "close";
    std::string out_string = writer.write(root);

	type_ = "close";
	std::cout << "closing session  " << session << std::endl;
	if(online){
	 	//Sending data to close the session with the session manager
		boost::network::http::client::request request_(endpoint_ + std::to_string(session_));
		response_ = client_.post(request_, out_string);
		string_stream_ << body(response_);
		std::string client_response_ = string_stream_.str();
		std::cout << "Session manager resonse: " << client_response_ << std::endl;
		string_stream_.str(std::string());
	}
}