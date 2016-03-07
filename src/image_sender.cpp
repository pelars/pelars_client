#include "image_sender.h"

ImageSender::ImageSender(int session, std::string endpoint, std::string token): token_(token), sending_complete_(true){
    root_["id"] = session;
    root_["type"] = "image";
  	endpoint_ = endpoint + std::string("multimedia") + std::string("?token=") + token_;
}

void ImageSender::send(std::string & data, std::string type, std::string what, bool automatic, long time){
	sending_complete_ = false;
	root_["data"] = data;
	root_["mimetype"] = type;
	root_["view"] = what;
	root_["triggering"] = automatic ? "automatic" : "manual";
	std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
	root_["time"] = time != 0 ? time : (double)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count();

	out_string_ = writer_.write(root_);
	try
	{
		boost::network::http::client::request endpoint(endpoint_);
		response_ = client_.put(endpoint, out_string_);
		response_.status();
	}
	catch (std::exception& e)
	{
		sending_complete_ = true;
		std::cout << "\tError during the image upload: " << e.what() << std::endl;
	}
	sending_complete_ = true;
}