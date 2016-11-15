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


void sendImage(int session, const std::string & end_point, const std::string & token, 
	           std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc_kinect,
	           std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>> pc_trigger, bool upload){

	ImageSender image_sender(session, end_point, token, upload);

	std::shared_ptr<ImageFrame> frames = std::make_shared<ImageFrame>();
	std::shared_ptr<Trigger> trigger = std::make_shared<Trigger>();

	std::string now, name;

	std::string folder_name = std::string("../../images/snapshots_") + std::to_string(session);
	if(!boost::filesystem::exists(folder_name)){
		boost::filesystem::path dir(folder_name);
		boost::filesystem::create_directory(dir);
	}

	//Workaround: try to use signal handler

	while(!to_stop){

		while(pc_trigger->readNoWait(trigger) == false && !to_stop){
			sleep(1);
		}

		if(pc_kinect->readNoWait(frames)){

			now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(trigger->time_.time_since_epoch()).count());
			name = std::string(folder_name + "/" + frames->type + "_" + now + "_" + std::to_string(session) + ".jpg");

			if(frames->color.rows > 0){
				cv::imwrite(name, frames->color);

				if(online){
					std::ifstream in(name, std::ifstream::binary);
					in.unsetf(std::ios::skipws);
				    in.seekg(0, std::ios::end);
				    std::streampos fileSize = in.tellg();
				    in.seekg(0, std::ios::beg);
				    std::vector<char> data(fileSize);
					in.read(&data[0], fileSize);
					std::string image = base64_encode((unsigned char*)&data[0], (unsigned int)data.size());

					image_sender.send(image, "jpg", frames->type, trigger->automatic_, now);		
				}
			}
			
		}		
	}
}
