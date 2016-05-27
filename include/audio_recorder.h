
#include <iostream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unistd.h>
#include <sys/wait.h>

extern int audio_recorder_pid;

void audioRecorder(unsigned int session);
std::string exec(const char* cmd);