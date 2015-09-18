#include "ide_handler.h"

bool stop_socket = false;

void ideHandler(struct mg_mgr & mgr){
	
	while(!stop_socket)
		mg_mgr_poll(&mgr, 5000);

	// Stopping mongoose
	mg_mgr_free(&mgr);
	std::cout << "Mongoose stopped" << std::endl;
}