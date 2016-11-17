#include "image_sender.h"

ImageSender::ImageSender(int session, const std::string & endpoint, const std::string & token, bool upload): 
           					token_(token), sending_complete_(true){
    root_["id"] = session;
    root_["type"] = "image";
    root_["creator"] = "client";
  	endpoint_ = endpoint + (upload ? std::string("multimediaupload") : std::string("multimedia")) + std::string("?token=") + token_;

}

void ImageSender::send(std::string & data, std::string type, std::string what, bool automatic, const std::string & time){
	sending_complete_ = false;
	root_["data"] = data;
	root_["mimetype"] = type;
	root_["view"] = what;
	root_["trigger"] = automatic ? "automatic" : "manual";
	root_["time"] = time;

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


void encodeImage(const std::string & name, std::string & content){
	
	std::ifstream in(name, std::ifstream::binary);
	in.unsetf(std::ios::skipws);
    in.seekg(0, std::ios::end);
    std::streampos fileSize = in.tellg();
    in.seekg(0, std::ios::beg);
    std::vector<char> data(fileSize);
	in.read(&data[0], fileSize);
	content = base64_encode((unsigned char*)&data[0], (unsigned int)data.size());

}


void sendImage(int session, const std::string & end_point, const std::string & token, 
	           std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc_kinect,
	           std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>> pc_trigger, bool upload){

	ImageSender image_sender(session, end_point, token, upload);

	std::string name, base64_image, type;

	std::string folder_name = std::string("../../images/snapshots_") + std::to_string(session);
	if(!boost::filesystem::exists(folder_name)){
		boost::filesystem::path dir(folder_name);
		boost::filesystem::create_directory(dir);
	}

	while(!to_stop){

		std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
		std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>();

		pc_trigger->read(trigger);
		pc_kinect->read(frames);
		std::cout <<"SENDING IMAGE " << std::endl;

		if(!to_stop){

			name = std::string(folder_name + "/" + frames->type + "_" + time2string(trigger->time_) + "_" + std::to_string(session) + ".jpg");
			type = frames->type;

			if(frames->color.rows > 0){
				cv::imwrite(name, frames->color);

				if(online){
					encodeImage(name, base64_image);
					image_sender.send(base64_image, "jpg", frames->type, trigger->automatic_, time2string(trigger->time_));		
				}
			}
			
		}		
	}
	std::cout << "terminating image sender " << type << std::endl;
}
