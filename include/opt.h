#pragma once
#include <boost/asio.hpp>
#include <iostream>

// Asion communication service and Asio keep alive
extern boost::asio::io_service io;
extern struct timespec start;

void aliver(const boost::system::error_code& /*e*/);

void asiothreadfx();

// Calculate a time interval
double deltats(const struct timespec & a, const struct timespec & b);

