
#include <iostream>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <unistd.h>
#include <boost/filesystem.hpp>

void audioRecorder(unsigned int session);
std::string exec(const char* cmd);