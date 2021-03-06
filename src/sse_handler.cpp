
#include "sse_handler.h"

#ifdef HAS_CURL
size_t write_data(void *ptr, size_t size, size_t nmemb, Encapsule * enc) {
	enc->body_.append((char*)ptr, nmemb);
	std::vector<std::string> strs;
	boost::split(strs, enc->body_, boost::is_any_of("\n\n"));
	int str_size = strs.size();
	Json::Value root_;
	if(str_size > 2){
		for(int j = 0; j < str_size - 1; ++j){
			std::size_t tmp = strs[j].find("{");
			if(tmp != std::string::npos){
				strs[j].erase(0, tmp);
				enc->addTime();
				enc->addData(strs[j]);
				enc->prepareData();
				if(online) 
					io.post([&enc]() {
						enc->send();
					});
				enc->localSend();
				snapshot_table = true;
				snapshot_people = true;
				snapshot_screen = true;
			}
		}
		enc->body_ = strs[str_size - 1];	
	}
	return nmemb;
}

Http::Http()
{
	multi_handle = curl_multi_init();
	handle_count = 0;
}

Http::~Http()
{
	curl_multi_cleanup(multi_handle);
}

void Http::update()
{	
	curl_multi_perform(multi_handle, &handle_count);
}

void Http::addRequest(const char* uri, const std::string token, Encapsule * enc){
	CURL * curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_URL, uri);
	struct curl_slist * headerlist = NULL;

	headerlist = curl_slist_append(headerlist,(std::string("Authorization: Bearer ") + token).c_str());

	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, enc);

	// VERBOSE
	curl_multi_add_handle(multi_handle, curl);
}

void sseHandler(DataWriter & websocket){

	Http http;

	std::ifstream infile("../../data/sse.txt");

	if(!infile.is_open())
	{
		std::cout << "cannot open sse.txt file in \"../../data/sse.txt\"" << std::endl;
	}else{
		std::string name, url, token;
		while(infile >> name >> url >> token){
			std::cout << "Adding " << name << " url " << url << " token " << token << std::endl;
			http.addRequest(url.c_str(), token, new Encapsule(websocket, name));
		}
	}
	while(!to_stop){
		http.update();
	}
}
#endif
