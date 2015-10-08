#include "image_sender.h"

ImageSender::ImageSender(int session, std::string endpoint, std::string token): token_(token){
    root_["id"] = session;
    root_["type"] = "image";
  	endpoint_ = endpoint + std::string("multimedia") + std::string("?token=") + token_;
}

void ImageSender::send(std::string & data, std::string type){
	root_["data"] = data;
	root_["mimetype"] = type;

	std::string out_string = writer_.write(root_);
	try
	{
		boost::network::http::client::request endpoint(endpoint_);
		endpoint_ = std::string();
		boost::network::http::client::response response = client_.put(endpoint, out_string);
	}
	catch (std::exception& e)
	{
		std::cout << "\tError during the image upload: " << e.what() << std::endl;
	}
}