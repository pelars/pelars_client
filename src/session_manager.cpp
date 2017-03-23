#include "session_manager.h"


SessionManager::SessionManager(std::string endpoint, const bool test): endpoint_(endpoint), error_(false), test_(test)
{	
	std::cout << "Parsing the input data" << std::endl;
	auto eResult =  data_.LoadFile( "../../data/personal.xml" );
	if(eResult != 0){
		std::cout << "\tError parsing personal.xml or file not present in /data/" << std::endl;
		terminateMe();
		error_ = true;
	}else{
		std::cout << "\tParsed the input data" << std::endl;
		createUser();
	}
}	

int SessionManager::getNewSession(double time)
{
	if(!error_){
		// Error variables
		bool error = false;
		bool parsed_success = false;;

		// Json message and content
		root_.clear();
		root_["user_id"] = user_id_;
		root_["institution_name"] = data_.FirstChildElement("Root")->FirstChildElement("institution_name")->GetText();
		root_["institution_address"] = data_.FirstChildElement( "Root")->FirstChildElement("institution_address")->GetText();
		root_["namespace"] = data_.FirstChildElement("Root")->FirstChildElement("namespace")->GetText();
		
		if(test_)
			root_["description"] = "test";
		
		if(time != 0){
			root_["start"] = time;
		}
		std::string out_string = writer_.write(root_);

		std::cout << "Requesting session id " << std::endl;
		try{
			// Making the put request to create a new session
			boost::network::http::client::request endpoint(endpoint_ + std::string("session") + std::string("?token=") + token_);
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
			std::cout << "\tError during the session request: " << e.what()  << "; setting session id to -1" <<std::endl;
			error = true;
			online = false;
			session_ = -1;
		}
		if(!parsed_success){
			// Error during the parsing step
			std::cout << "\tError during the session request: Couldn't parse json message; setting session id to -1" <<std::endl;
			error = true;
			online = false;
			session_ = -1;
		}

		if(!error)
			std::cout << "\tGot session id " << session_ << std::endl;
		
		return session_;
	}
	else
		return -1;	
}


void SessionManager::login(){

	root_.clear();
	if(online){
		std::cout << "Requesting login " << std::endl;
		try{
			//std::cout << endpoint_ + std::string("password") + std::string("?user="+mail_+"&pwd="+password_) << std::endl;
			boost::network::http::client::request request(endpoint_ + std::string("password") + std::string("?user="+mail_+"&pwd="+password_));
			std::cout << endpoint_ + std::string("password") + std::string("?user="+mail_+"&pwd="+password_) << std::endl;
			boost::network::http::client::response response = client_.post(request);
			std::cout << response.body() << std::endl;

			if(reader_.parse(response.body(), root_)){
				token_ = root_["token"].asString();
				std::cout << "\tSuccess" << std::endl;
				std::cout << "\tToken: " << token_ << std::endl;
			}else{
				std::cout << "Failed" << std::endl;
			}
		}catch (std::exception& e){
			// Generic error during the request
			std::cout << "\tError during the login request: " << e.what()  << "; setting offline mode" <<std::endl;
			online = false;
		}
	}
	
}

void SessionManager::createUser(){
	// Creating the new user

	std::cout << "Creating user" <<std::endl;

	root_.clear();
	root_["name"] = data_.FirstChildElement("Root")->FirstChildElement("name")->GetText();
	root_["password"] = data_.FirstChildElement("Root")->FirstChildElement("password")->GetText();
	root_["affiliation"] = data_.FirstChildElement("Root")->FirstChildElement("affiliation")->GetText();
	root_["namespace" ] = data_.FirstChildElement("Root")->FirstChildElement("namespace")->GetText();
	root_["email" ] = data_.FirstChildElement("Root")->FirstChildElement("email")->GetText();
	mail_ = root_["email" ].asString();
	password_ = root_["password"].asString();
	std::string out_string = writer_.write(root_);
	try{
		// Making the put request to create a new session
		boost::network::http::client::request endpoint(endpoint_ + std::string("user"));
		response_ = client_.put(endpoint, out_string);
		session_manager_response_ = response_.body();
		// Read the response and parse the json message to get the session id
		bool parsedSuccess = reader_.parse(session_manager_response_, root_);
		if(parsedSuccess)
			user_id_ = root_["id"].asInt();
		if(root_["message"].asString().find(std::string("User already present")) == std::string::npos)
			std::cout << "\tGot response " << session_manager_response_;
	}
	catch (std::exception& e){
		// Generic error during the request
		std::cout << "\tError during the user creation: " << e.what()  <<std::endl;
	}
}

void SessionManager::closeSession(int session, double time)
{
	// Close the session and exit
	root_.clear();

	// Json message
	root_["op_code"] = "close";
	if(time != 0.0)
		root_["time"] = time;
	std::string out_string = writer_.write(root_);

	std::cout << "closing session " << session << std::endl;
	if(online){
		//Sending data to close the session with the session manager
		boost::network::http::client::request request_(endpoint_ + std::string("session/") + std::to_string(session) + std::string("?token=") + token_);
		response_ = client_.post(request_, out_string);
		std::cout << "\tSession manager response: " << response_.body();
	}
}

std::string SessionManager::getToken(){
	return token_;
}





