#pragma once
#include "mongoose.h"
#include <iostream>
#include "opt.h"

extern bool to_stop;

void ideHandler(DataWriter & websocket, const char * endpoint);
void ev_handler(struct mg_connection * nc, int ev, void * ev_data);