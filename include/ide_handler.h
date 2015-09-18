#pragma once
#include "mongoose.h"
#include <iostream>

extern bool stop_socket;

void ideHandler(struct mg_mgr & mgr);