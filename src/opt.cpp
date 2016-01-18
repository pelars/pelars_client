#include "opt.h"

// To stop all the threads if one receives a stop signal
bool to_stop = false;
// Take a camera snapshot of the table
bool snapshot_table = false;
// Take a camera snapshot of the people
bool snapshot_people = false;
// Take a snapshot of the screen
bool snapshot_screen = false;
// Connection status
bool online = true;
// Enable visualization
bool visualization = false;
// System starting time
const std::string currentDateTimeNow = currentDateTime();
// sending interval
double interval = 1000;

// Keep alive
void aliver(const boost::system::error_code& /*e*/)
{
	
}

// Signal handler
void sig_handler(int signum)
{
		to_stop = true;
		printf("Received signal %d\n", signum);
}

void asiothreadfx()
{
	boost::asio::deadline_timer t(io, boost::posix_time::seconds(100000));
	t.async_wait(aliver);
	io.run();
}

const std::string currentDateTime() {
	time_t     now = time(0);
	struct tm  tstruct;
	char       buf[80];
	tstruct = *localtime(&now);
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
	return buf;
}

// Check if escape was pressed to terminate execution
void checkEscape(bool visualization, bool special){
	if(!visualization && !special){
		while(std::cin.get() != 27){
			if (to_stop){
				break;
			}
		}
		to_stop = true;
		std::cout << "Stopping" << std::endl;
	}
}

