#pragma once
#include "mongoose.h"
#include <iostream>
#include "opt.h"

extern bool to_stop;
extern bool snapshot_table;
extern bool snapshot_people;
extern bool snapshot_screen;

void ideHandler(DataWriter & websocket, const char * endpoint, const char * endpoint2);
void ev_handler(struct mg_connection * nc, int ev, void * ev_data);
void button_handler(struct mg_connection * nc, int ev, void * ev_data);