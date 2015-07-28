#include "sse_handler.h"

/*
void Sse_handler::add_sse(std::string name, std::string url, std::string token){
	std::cout << "Adding " << name << " url " << url << " token " << token << std::endl;
	
	curlpp::Easy request;
	//curlpp::types::WriteFunctionFunctor functor(call_back);
	//curlpp::options::WriteFunction *test = new curlpp::options::WriteFunction(functor);
	//request.setOpt(test);
	std::list<std::string> header; 
    header.push_back("Authorization: Bearer " + token);
    request.setOpt(new curlpp::options::HttpHeader(header));  

	// Setting the URL to retrive.
	request.setOpt(new curlpp::options::Url(url));
	request.setOpt(new curlpp::options::Verbose(false));

	request.perform();
}


Sse_handler::Sse_handler(){
	std::ifstream infile("../../data/sse.txt");

	if(!infile.is_open())
  	{
    	std::cout << "cannot open sse.txt file in \"../../data/sse.txt\"" << std::endl;
  	}else{

	  	std::string name, url, token;
	  	
	  	while(infile){
	  		infile >> name >> url >> token;
	  		add_sse(name, url, token);
	  	}
  	}
}



*/

#include "sse_handler.h"

extern bool to_stop;

size_t write_data(void *ptr, size_t size, size_t nmemb, std::string *body) {
    std::cout << "received "  << std::string((char*)ptr, nmemb) << std::endl;
    body->append((char*)ptr, nmemb);
    return nmemb;
}


Http::Http(DataWriter & websocket): websocket_(websocket)
{
	fp = fopen("test.txt","wb");
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
	std::cout << "BODY " << body << std::endl;
}

void Http::addRequest(const char* uri, std::string token, std::string * body){
	CURL * curl = curl_easy_init();
	curl_easy_setopt(curl, CURLOPT_URL, uri);
	struct curl_slist * headerlist = NULL;

	std::cout <<(std::string("Authorization: Bearer ") + token).c_str() << std::endl;

	headerlist = curl_slist_append(headerlist,(std::string("Authorization: Bearer ") + token).c_str());

	curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_data);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, body);

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
	 	  	http.body_vector_.push_back(std::string());
			http.addRequest(url.c_str(), token, &(http.body_vector_.back()));
	  	}
  	}

  	while(!to_stop)
  		http.update();
}

