#include "aliver.h"

void keep_alive(int session, std::string url){

	DataWriter alive_socket(url, session); 
	std::cout << "opened aliver on " + url << std::endl;

	Json::Value root;
	Json::StyledWriter writer;
	root["status"] = "alive";
	std::string alive_message = writer.write(root);

	while(online && !to_stop){
		alive_socket.writeData(alive_message);
		sleep(1);
	}

	alive_socket.astop();
}