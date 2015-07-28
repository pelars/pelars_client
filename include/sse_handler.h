#pragma once
#include <curlpp/cURLpp.hpp>
#include <curlpp/Easy.hpp>
#include <curlpp/Options.hpp>
#include <curlpp/Exception.hpp>
#include <curlpp/Infos.hpp>
#include <string.h>
#include <fstream>
#include <vector>
#include <thread>
#include "data_writer.h"

void sse_handler(DataWriter & websocket);

class Http
{
	CURLM* multi_handle; 
	int handle_count;
	FILE * fp;
	DataWriter & websocket_;


	public:

	Http(DataWriter & websocket);
	~Http();
	void update();
	void addRequest(const char* uri, std::string token, std::string * body);
	std::vector<std::string> body_vector_;

};