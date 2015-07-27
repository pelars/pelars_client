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

void sse_handler();

class Http
{
	CURLM* multi_handle; 
	int handle_count;
	CURL* curl = NULL;


	public:
	Http();
	~Http();
	void update();
	void addRequest(const char* uri, std::string token);
};