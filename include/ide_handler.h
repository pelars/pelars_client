#pragma once
#include "mongoose.h"
#include "ide_trigger.h"

void ideHandler(IdeTrigger & websocket, const char * endpoint, const char * endpoint2);
void ev_handler(struct mg_connection * nc, int ev, void * ev_data);
void button_handler(struct mg_connection * nc, int ev, void * ev_data);