#pragma once
#include <ctime>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <data_writer.h>
#include "opt.h"

using boost::asio::ip::udp; //TODO orribile
extern bool to_stop;


class UdpServer{
  
public:
    UdpServer(boost::asio::io_service& io_service, DataWriter &websocket)
    : socket_(io_service, udp::endpoint(udp::v4(), 6789)), websocket_(websocket)
    {
    	receive();
    }

private:
	void receive();
  	void handleReceive(const boost::system::error_code& error, std::size_t);

  	udp::socket socket_;
  	udp::endpoint remote_endpoint_;
  	boost::array<char, 1024> recv_buffer_;
  	DataWriter & websocket_;
};

void keyLogger(DataWriter & websocket, int session, boost::asio::io_service & io_service);