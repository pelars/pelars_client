#include "opt.h"


void aliver(const boost::system::error_code& /*e*/)
{
	
}
void asiothreadfx()
{
    boost::asio::deadline_timer t(io, boost::posix_time::seconds(100000));
    t.async_wait(aliver);
    io.run();

}

// Calculate a time interval
double deltats(const struct timespec & a, const struct timespec & b)
{
  return a.tv_sec-b.tv_sec + ((a.tv_nsec - b.tv_nsec) / 1000000000.0);
}

const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}
