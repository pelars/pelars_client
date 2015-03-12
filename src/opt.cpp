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

