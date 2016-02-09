#include "data_writer.h"

DataWriter::DataWriter(const std::string & uri, const int session, const bool create_file): uri_(uri+std::string("/")+std::to_string(session)), m_open_(false), m_done_(false), session_(session)
{
	m_client_.clear_access_channels(websocketpp::log::alevel::all);
	m_client_.set_access_channels(websocketpp::log::alevel::connect);
	m_client_.set_access_channels(websocketpp::log::alevel::disconnect);
	m_client_.set_access_channels(websocketpp::log::alevel::app);

	m_client_.set_close_handler(bind(&DataWriter::onClose, this,  std::placeholders::_1));
	m_client_.set_fail_handler(bind(&DataWriter::onFail, this, std::placeholders::_1));

	if(uri_.find("http://") == 0){
		// Initialize the Asio transport policy
		m_client_.init_asio();
		con_ = m_client_.get_connection(uri_, ec_);
		m_hdl_ = con_->get_handle();
		m_client_.connect(con_);
		websocketpp::lib::thread asio_thread(&client::run, &m_client_);
		thread_.swap(asio_thread);
		failed_ = false;
	}
	else
	{
		online = false;
		failed_ = true;
		std::cout << "Invalid collector endpoint " << uri_ << std::endl;
	}

	if(create_file){
		file_name_ = std::string("../../backup/session_")+ std::to_string(session);
		file_extention_ = std::string(".txt");
		complete_file_name_ = file_name_ + file_extention_;
		fs_.open(complete_file_name_);
	}
}

void DataWriter::writeData(const std::string s)
{ 
	if(con_ && online)
		m_client_.send(m_hdl_, s , websocketpp::frame::opcode::text, ec_);
}

void DataWriter::stop()
{
	//online = false;
	std::cout << "stop requested from DataWriter\n";
	if(!failed_){
		m_client_.stop();
		thread_.join();
	}
}

void DataWriter::onClose(websocketpp::connection_hdl) {
	std::cout << "close requested from on_close\n";
	online = false;
}

int DataWriter::getSession() {
	return session_;
}

void DataWriter::onFail(websocketpp::connection_hdl) {
	std::cout << "stop requested from on_fail\n";
	failed_ = true;
	online = false;
	stop();
}

void DataWriter::writeLocal(const std::string s) {
	m_.lock();
	fs_ << s.size() << s;
	m_.unlock();

}

DataWriter::~DataWriter() {
	fs_.close();
	
}


