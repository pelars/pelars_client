#pragma once
#include "opt.h"

extern bool to_stop;

class UdpServer{
	
public:
		UdpServer(boost::asio::io_service& io_service, DataWriter &websocket)
		: socket_(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 6789)), websocket_(websocket)
		{
			receive();
		}

private:
	void receive();
		void handleReceive(const boost::system::error_code& error, std::size_t);

		boost::asio::ip::udp::socket socket_;
		boost::asio::ip::udp::endpoint remote_endpoint_;
		boost::array<char, 1024> recv_buffer_;
		DataWriter & websocket_;
};

void keyLogger(DataWriter & websocket, boost::asio::io_service & io_service);