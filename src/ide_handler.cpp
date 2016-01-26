#include "ide_handler.h"

void ideHandler(DataWriter & websocket, const char * endpoint,  const char * endpoint2)
{
	// Mongoose websocket listener and message structure
	struct mg_mgr mgr;
	struct mg_connection * nc;
	struct mg_connection * nc2;
	//MiniEncapsule writer(websocket);
	mg_mgr_init(&mgr, &websocket);
	nc = mg_bind(&mgr, endpoint, ev_handler);
	nc2 = mg_bind(&mgr, endpoint2, button_handler);
	mg_set_protocol_http_websocket(nc);
	mg_set_protocol_http_websocket(nc2);
	std::cout << "Mongoose websocket started on port " << std::string(endpoint) << " for the arduino ide" << std::endl;
	std::cout << "Mongoose websocket started on port " << std::string(endpoint2) << " for the buttons " << std::endl;
	
	while(!to_stop)
		mg_mgr_poll(&mgr, 5000);

	mg_mgr_free(&mgr);
	std::cout << "Mongoose stopped" << std::endl;
}

void ev_handler(struct mg_connection * nc, int ev, void * ev_data)
{
	switch (ev) {
		case NS_WEBSOCKET_FRAME:
			{
				DataWriter * writer = (DataWriter *)nc->mgr->user_data;
				struct websocket_message * wm = (struct websocket_message *)ev_data;
				std::string message((char *)(wm->data), wm->size);
				
				//std::cout << "SENDING " << message << std::endl;
				if(online){
						io.post( [&writer, message]() {
							writer->writeData(message);
							});
						}
				writer->writeLocal(message);
			}
			break;
		default:
			break;
	}
}

void button_handler(struct mg_connection * nc, int ev, void * ev_data)
{
	switch (ev) {
		case NS_WEBSOCKET_FRAME:
			{
				//std::cout << "pressed button" << std::endl;
				DataWriter * writer = (DataWriter *)nc->mgr->user_data;
				struct websocket_message * wm = (struct websocket_message *)ev_data;
				std::string message((char *)(wm->data), wm->size);
				snapshot_table = true;
				snapshot_people = true;
				snapshot_screen = true;
				//std::cout << "SENDING " << message << std::endl;
				if(online){
						io.post( [&writer, message]() {
							writer->writeData(message);
							});
						}
				writer->writeLocal(message);
			}
			break;
		default:
			break;
	}
}