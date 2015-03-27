#include "data_writer.h"

DataWriter::DataWriter(const std::string & uri): uri_(uri), m_open_(false), m_done_(false)
{
  m_client_.clear_access_channels(websocketpp::log::alevel::all);
  m_client_.set_access_channels(websocketpp::log::alevel::connect);
  m_client_.set_access_channels(websocketpp::log::alevel::disconnect);
  m_client_.set_access_channels(websocketpp::log::alevel::app);

  m_client_.set_close_handler(bind(&DataWriter::on_close,this,::_1));
  m_client_.set_fail_handler(bind(&DataWriter::on_fail,this,::_1));

  // Initialize the Asio transport policy
  m_client_.init_asio();
  con_ = m_client_.get_connection(uri_, ec_);
  std::cout << "connection result: " << ec_.message() << " " << con_ << std::endl;
  m_hdl_ = con_->get_handle();
  m_client_.connect(con_);
  websocketpp::lib::thread asio_thread(&client::run, &m_client_);
  thread_.swap(asio_thread);
}

void 
DataWriter::writeData(const std::string s)
{ 
  if(con_ && online)
    m_client_.send(m_hdl_, s , websocketpp::frame::opcode::text, ec_);
  else
  {
    std::cout << "writing to file !!!!!" << std::endl;
  }
}

void 
DataWriter::stop()
{
  online = false;
  std::cout << "stop requested from DataWriter\n";
  m_client_.stop();
  thread_.join();
}

void DataWriter::on_close(websocketpp::connection_hdl) {
  std::cout << "stop requested from on_close\n";
  online = false;

}

void DataWriter::on_fail(websocketpp::connection_hdl) {
  std::cout << "stop requested from on_fail\n";
  stop();
}

