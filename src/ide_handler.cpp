#include "ide_handler.h"

void ideHandler(struct mg_mgr & mgr){
	
	while(!to_stop)
		mg_mgr_poll(&mgr, 5000);
}