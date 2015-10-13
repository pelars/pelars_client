#include "ide_handler.h"

void ideHandler(struct mg_mgr & mgr){
	
	while(!to_stop)
		mg_mgr_poll(&mgr, 5000);

	// Stopping mongoose
	mg_mgr_free(&mgr);
	std::cout << "Mongoose stopped" << std::endl;
}

void ev_handler(struct mg_connection * nc, int ev, void * ev_data)
{
	switch (ev) {
	case NS_WEBSOCKET_FRAME:
	{
		MiniEncapsule * writer = (MiniEncapsule *)nc->mgr->user_data;
		struct websocket_message * wm = (struct websocket_message *)ev_data;
		std::string message((char *)(wm->data), wm->size);
		writer->parse(message);
		std::cout << "SENDING " << writer->out_message_ << std::endl;
		if(online){
				io.post( [&writer]() {
					writer->websocket_.writeData(writer->out_message_);
					});
				}
		writer->websocket_.writeLocal(writer->out_message_);
	}
	break;
		default:
	break;
	}
}