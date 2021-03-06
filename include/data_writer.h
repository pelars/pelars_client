#pragma once
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <fstream>
#include <mutex>

#define BOOST_BIND_NO_PLACEHOLDERS

extern bool online;

class DataWriter
{
public:
	typedef websocketpp::client<websocketpp::config::asio_client> client;
	typedef websocketpp::lib::lock_guard<websocketpp::lib::mutex> scoped_lock;
	std::string file_name_, file_extention_, complete_file_name_;

	DataWriter(const std::string & uri, const int session, const bool create_file = true);
	DataWriter(const std::string & uri, const int session, const int aliver);

	~DataWriter();
	void stop();
	void astop();
	void writeLocal(const std::string s);
	void writeData(const std::string s);
	int getSession();
	std::string getUri() const;

private:

	const std::string uri_;
	client m_client_;
	websocketpp::connection_hdl m_hdl_;
	bool m_open_;
	bool m_done_;
	bool failed_;
	std::vector<std::string> * buffer_;
	client::connection_ptr con_;
	websocketpp::lib::error_code ec_;
	websocketpp::error::category cat_;
	websocketpp::lib::thread thread_;
	std::mutex m_;
	std::ofstream fs_;
	int session_;
	
	void onClose(websocketpp::connection_hdl);
	void onFail(websocketpp::connection_hdl);
	void aonFail(websocketpp::connection_hdl);

};
