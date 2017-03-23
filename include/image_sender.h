#pragma once
#include <string>
#include <boost/network/protocol/http/client.hpp>
#include <json/json.h>
#include "trigger.h"
#include "image_frame.h"

extern bool online;
extern std::mutex synchronizer;


class ImageSender{
	
public:

	ImageSender(int session, const std::string & end_point, const std::string & token, bool upload = false);
	void send(std::string & data, std::string type, std::string what, bool automatic, const std::string & time);
	operator bool() const { return sending_complete_;}

private:
	std::string endpoint_, token_;
    boost::network::http::client client_;
	boost::network::http::client::response response_;
	Json::Value root_;
	Json::StyledWriter writer_;
	std::string out_string_;
	bool sending_complete_;
};

void sendImage(int session, const std::string & end_point, const std::string & token, 
	           std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pc_kinect,
	           std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>> pc_trigger, bool upload);

void sendScreenshot(int session, const std::string & end_point, const std::string & token, 
	           std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>> pc_trigger, bool upload);

void encodeImage(const std::string & name, std::string & encode);