#include "image_sender.h"

ImageSender::ImageSender(int session, std::string endpoint, std::string token): token_(token){
    root_["id"] = session;
    root_["type"] = "image";
  	endpoint_ = endpoint + std::string("multimedia") + std::string("?token=") + token_;
}

void ImageSender::send(std::string & data, std::string type){
	root_["data"] = data;
	root_["mimetype"] = type;

	out_string_ = writer_.write(root_);
	try
	{
		boost::network::http::client::request endpoint(endpoint_);
	
		response_ = client_.put(endpoint, out_string_);
		response_.status();
	}
	catch (std::exception& e)
	{
		std::cout << "\tError during the image upload: " << e.what() << std::endl;
	}
}