#pragma once
#include <signal.h>
#include <iostream>

extern bool to_stop;

void sig_handler(int signum);
void terminateMe();
void initTermination();