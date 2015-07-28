#include "session_manager.h"


SessionManager::SessionManager(std::string endpoint): endpoint_(endpoint)
{	
	srand (time(NULL));
	std::cout << "Parsing the input data" << std::endl;
    tinyxml2::XMLError eResult =  data_.LoadFile( "../../data/personal.xml" );
    if(eResult != 0){
    	std::cout << "\tError parsing personal.xml or file not present in /data/" << std::endl;
    	to_stop = true;
    }else
    	std::cout << "\tParsed the input data" << std::endl;

   createUser();
}	


int SessionManager::getNewSession()
{
	// Error variables
    bool error = false;
    bool parsed_success;

    // Json message and content
    root_.clear();
    root_["user_id"] = user_id_;
    root_["institution_name"] = data_.FirstChildElement( "Root" )->FirstChildElement( "institution_name" )->GetText();
    root_["institution_address"] = data_.FirstChildElement( "Root" )->FirstChildElement( "institution_address" )->GetText();
    root_["namespace" ] = data_.FirstChildElement( "Root" )->FirstChildElement( "namespace" )->GetText();
    std::string out_string = writer_.write(root_);

	std::cout << "Requesting session id " << std::endl;
	try{
		// Making the put request to create a new session
		boost::network::http::client::request endpoint(endpoint_ + std::string("session"));
		response_ = client_.put(endpoint, out_string);
		session_manager_response_ = response_.body();
		//std::cout << session_manager_response_ << std::endl;

		// Read the response and parse the json message to get the session id
		session_manager_response_.erase(std::remove(session_manager_response_.begin(), session_manager_response_.end(), '\n'), session_manager_response_.end());
		root_.clear();
		Json::Reader reader;
		parsed_success = reader.parse(session_manager_response_, root_, false);
		session_ = root_["id"].asInt();
	}
	catch (std::exception& e){
		// Generic error during the request
		std::cout << "\tError during the session request: " << e.what()  << "; setting session id to 0" <<std::endl;
		error = true;
		online = false;
		session_ = -1;
	}
	if(!parsed_success){
		// error during the parsing step
		std::cout << "\tError during the session request: Couldn't parse json message; setting session id to 0" <<std::endl;
		error = true;
		online = false;
		session_ = -1;
	}

	if (!error)
		std::cout << "\tGot session id " << session_ << std::endl;
	
	return session_;
}

void SessionManager::createUser(){
	// Creating the new user

	std::cout << "creating user" <<std::endl;

	root_.clear();
    root_["name"] = data_.FirstChildElement( "Root" )->FirstChildElement( "name" )->GetText();
    root_["password"] = data_.FirstChildElement( "Root" )->FirstChildElement( "password" )->GetText();
    root_["affiliation"] = data_.FirstChildElement( "Root" )->FirstChildElement( "affiliation" )->GetText();
    root_["namespace" ] = data_.FirstChildElement( "Root" )->FirstChildElement( "namespace" )->GetText();
    root_["email" ] = data_.FirstChildElement( "Root" )->FirstChildElement( "email" )->GetText();
    std::string out_string = writer_.write(root_);

    try{
		// Making the put request to create a new session
		boost::network::http::client::request endpoint(endpoint_ + std::string("user"));
		response_ = client_.put(endpoint, out_string);
		session_manager_response_ = response_.body();
		// Read the response and parse the json message to get the session id
		//session_manager_response.erase(std::remove(session_manager_response.begin(), session_manager_response.end(), '\n'), session_manager_response.end());
		bool parsedSuccess = reader_.parse(session_manager_response_, root_);
		user_id_ = root_["id"].asInt();

		if(root_["message"].asString().find(std::string("User already present")) != std::string::npos)
			std::cout << "\tGot response " << session_manager_response_;
	}
	catch (std::exception& e){
		// Generic error during the request
		std::cout << "\tError during the user creation: " << e.what()  <<std::endl;
	}

}

void SessionManager::closeSession(int session)
{
	// Close the session and exit
  	// Preaparing data to close the session
  	root_.clear();

    // Json message
    root_["op_code"] = "close";
    std::string out_string = writer_.write(root_);

	std::cout << "closing session " << session << std::endl;
	if(online){
	 	//Sending data to close the session with the session manager
		boost::network::http::client::request request_(endpoint_ + std::string("session/") + std::to_string(session));
		response_ = client_.post(request_, out_string);
		std::cout << "\tSession manager response: " << response_.body();
	}
}



