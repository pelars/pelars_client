#include "aliver.h"

// Sends a periodic (1s) message as keep alive to the server
void keep_alive(DataWriter & websocket){

	Json::Value root;
	Json::StyledWriter writer;
	root["status"] = "alive";
	std::string alive_message = writer.write(root);

	while(online && !to_stop)
	{
		io.post([&websocket, alive_message]() {
				websocket.writeData(alive_message);
				});
		sleep(1);
	}

	websocket.astop();
}