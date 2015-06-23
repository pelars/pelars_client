#include "session_manager.h"


SessionManager::SessionManager(std::string endpoint): endpoint_(endpoint)
{	
	srand (time(NULL));
	std::cout << "parsing the input data" << std::endl;
    tinyxml2::XMLError eResult =  data_.LoadFile( "../../data/personal.xml" );
    if(eResult != 0){
    	std::cout << "error parsing personal.xml or file not present in /data/" << std::endl;
    	to_stop = true;
    }else
    	std::cout << "parsed the input data" << std::endl;

   createUser();
}	


int SessionManager::getNewSession()
{
	// Error variables
    bool error = false;
    bool parsed_success;

    // Json message and content
    Json::Value root;
    Json::StyledWriter writer;
    root["teacher_name"] = data_.FirstChildElement( "Root" )->FirstChildElement( "name" )->GetText();
    root["institution_name"] = data_.FirstChildElement( "Root" )->FirstChildElement( "institution_name" )->GetText();
    root["institution_address"] = data_.FirstChildElement( "Root" )->FirstChildElement( "institution_address" )->GetText();
    root["namespace" ] = data_.FirstChildElement( "Root" )->FirstChildElement( "namespace" )->GetText();
    std::string out_string = writer.write(root);

	std::cout << "Requesting session id " << std::endl;
	try{
		// Making the put request to create a new session
		boost::network::http::client::request endpoint(endpoint_ + std::string("session"));
		response_ = client_.put(endpoint, out_string);
		string_stream_ << body(response_);
		session_manager_response_ = string_stream_.str();

		// Read the response and parse the json message to get the session id
		std::cout << "got response " << session_manager_response_ <<std::endl;
		session_manager_response_.erase(std::remove(session_manager_response_.begin(), session_manager_response_.end(), '\n'), session_manager_response_.end());
		Json::Value root;
		Json::Reader reader;
		parsed_success = reader.parse(session_manager_response_, root, false);
		session_ = root["id"].asInt();
	}
	catch (std::exception& e){
		// Generic error during the request
		std::cout << "Error during the session request: " << e.what()  << "; setting session id to 0" <<std::endl;
		error = true;
		online = false;
		session_ = -1;
	}
	if(!parsed_success){
		// error during the parsing step
		std::cout << "Error during the session request: Couldn't parse json message; setting session id to 0" <<std::endl;
		error = true;
		online = false;
		session_ = -1;
	}

	if (!error)
		std::cout << "Got session id " << session_ << std::endl;
	
	return session_;
}

void SessionManager::createUser(){
	std::cout << "creating user" <<std::endl;
	// Creating the new user
    Json::Value root;
    Json::StyledWriter writer;
    root["name"] = data_.FirstChildElement( "Root" )->FirstChildElement( "name" )->GetText();
    root["role"] = data_.FirstChildElement( "Root" )->FirstChildElement( "role" )->GetText();
    root["affiliation"] = data_.FirstChildElement( "Root" )->FirstChildElement( "affiliation" )->GetText();
    root["namespace" ] = data_.FirstChildElement( "Root" )->FirstChildElement( "namespace" )->GetText();
    std::string out_string = writer.write(root);

    try{
		// Making the put request to create a new session
		boost::network::http::client::request endpoint(endpoint_ + std::string("user"));
		boost::network::http::client::response response = client_.put(endpoint, out_string);
		std::stringstream string_stream;
		string_stream << body(response);
		std::string session_manager_response = string_stream.str();

		// Read the response and parse the json message to get the session id
		session_manager_response.erase(std::remove(session_manager_response.begin(), session_manager_response.end(), '\n'), session_manager_response.end());
		Json::Value root;
		Json::Reader reader;
		reader.parse(session_manager_response_, root, false);
		std::string message = root["message"].asString();
		if(message.find(std::string("User already present")) != std::string::npos)
			std::cout << "got response from user creator " << session_manager_response <<std::endl;
	}
	catch (std::exception& e){
		// Generic error during the request
		std::cout << "Error during the user creation: " << e.what()  <<std::endl;
	}

}

void SessionManager::closeSession(int session)
{
	// Close the session and exit
  	// Preaparing data to close the session
  	Json::Value root;
    Json::StyledWriter writer;
    string_stream_.str(std::string());

    // Json message
    root["op_code"] = "close";
    std::string out_string = writer.write(root);

	std::cout << "closing session " << session << std::endl;
	if(online){
	 	//Sending data to close the session with the session manager
		boost::network::http::client::request request_(endpoint_ + std::string("session/") + std::to_string(session));
		response_ = client_.post(request_, out_string);
		string_stream_ << body(response_);
		std::string client_response_ = string_stream_.str();
		std::cout << "Session manager response: " << client_response_ << std::endl;
		string_stream_.str(std::string());
	}
}



