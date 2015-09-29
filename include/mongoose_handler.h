#pragma once
#include "mongoose.h"
#include "opt.h"
#include <iostream>
#include <json/json.h>

extern bool stop_socket;

void ev_handler(struct mg_connection * nc, int ev, void * ev_data);