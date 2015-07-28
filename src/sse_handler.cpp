
#include "sse_handler.h"

extern bool to_stop;
extern bool online;

size_t write_data(void *ptr, size_t size, size_t nmemb, encapsule * enc) {
    //std::cout << "received " << enc->name_ << " " << std::string((char*)ptr, nmemb) << std::endl;
    enc->body_.append((char*)ptr, nmemb);
    std::vector<std::string> strs;
	boost::split(strs, enc->body_, boost::is_any_of("\n\n"));
	int str_size = strs.size();
	if(str_size > 2){
		for(int j = 0; j < str_size - 1; ++j){
			std::size_t tmp = strs[j].find("{");
			if(tmp != std::string::npos){
				strs[j].erase(0, tmp);
				enc->addData(strs[j]);
				//std::cout << "sending " << enc->to_send_ << std::endl;
				if(online) 
	          		io.post( [&enc]() {
	                	enc->send();
	                });
		        enc->localSend();
		    }
	    }
	    enc->body_ = strs[str_size - 1];	
	}
    return nmemb;
}


Http::Http(DataWriter & websocket): websocket_(websocket)
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

void Http::addRequest(const char* uri, const std::string token, encapsule * enc){
	CURL * curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_URL, uri);
	struct curl_slist * headerlist = NULL;

	std::cout <<(std::string("Authorization: Bearer ") + token).c_str() << std::endl;

	headerlist = curl_slist_append(headerlist,(std::string("Authorization: Bearer ") + token).c_str());

	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, enc);

	// VERBOSE
	//curl_easy_setopt(curl, CURLOPT_VERBOSE, 1L);
	curl_multi_add_handle(multi_handle, curl);
}

void sse_handler(DataWriter & websocket){

	Http http(websocket);

	std::ifstream infile("../../data/sse.txt");

	if(!infile.is_open())
  	{
    	std::cout << "cannot open sse.txt file in \"../../data/sse.txt\"" << std::endl;
  	}else{

	  	std::string name, url, token;
	  	while(infile >> name >> url >> token){
	 	  	std::cout << "Adding " << name << " url " << url << " token " << token << std::endl;
	 	  	http.enc_vector_.push_back(encapsule(websocket, name));
			http.addRequest(url.c_str(), token, &(http.enc_vector_.back()));
	  	}
  	}

  	while(!to_stop)
  		http.update();
}

