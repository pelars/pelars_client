#include <boost/filesystem.hpp>
#include <string>
#include "session_manager.h"
#include "image_sender.h"
#include <base64.h>
#include "data_writer.h"
#include "opt.h"

int uploadData(std::string & file_name,const std::string & end_point, int session_id = 0);