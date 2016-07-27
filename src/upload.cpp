#include "upload.h"

int uploadData(std::string & file_name, const std::string & end_point, int session_id){

	int packet_size = 0;
	char buffer[1024];
	int num = 0;
	double time = 0;
	Json::Value root;
	Json::Reader reader;

	// Remove path prefix to get only name
	std::size_t found = file_name.find_last_of("/");
	if(found != std::string::npos)
		file_name = file_name.substr(found + 1);

	// Open file
	std::ifstream in("../../backup/" + file_name);
	if(!in.is_open()){
		std::cout << "could not open file " << std::endl;
		return -1;
	}

	// Read the session id from the file name
	std::string delimiter = "_";
	size_t last = 0; 
	size_t next = file_name.find(delimiter, last); 
	last = next + 1;
	next = file_name.find(delimiter, last);
	int s = stoi(file_name.substr(last, next - last));
	//bool new_session = false;

	SessionManager sm(end_point, false);
	sm.login();

	if(session_id != 0){
		s = session_id;
	}else{
		// Create new session if the session id is not present on the server
		if(s == -1){
			// Read first packet and get time to create the session with the correct time if it was not opened
			in >> packet_size;
			in.read(buffer, packet_size);
			std::string tmp(buffer, packet_size);

			reader.parse(tmp, root, false);
			if(root["obj"].isArray()){
				for(Json::Value & a : root["obj"])
					time = a["time"].asInt64();
			}
			else{
				time = root["obj"]["time"].asInt64();
			}
			
			in.seekg(0);
			//new_session = true;
			s = sm.getNewSession(time);
		}
	}
	// Connect the websocker on the upload endpoint
	DataWriter collector(end_point + "upload", s, false);
	sleep(5);

	// Read data and send it in chunks of 300 packets every 10s
	
	while(!in.eof()){
		in >> packet_size;
		in.read(buffer, packet_size);
		std::string tmp(buffer, packet_size);
		collector.writeData(tmp);
		std::cout << "." << std::flush;
		num++;
		if(!(num % 100)){
			std::cout << "sent 100 packets, sleeping 2s" << std::endl;
			sleep(2);
		}
	}

	if(root["obj"].isArray()){
		for(Json::Value & a : root["obj"])
			time = a["time"].asInt64();
	}
	else{
		time = root["obj"]["time"].asInt64();
	}
		
	in.close();
	std::cout << std::endl;
	std::cout << num << " packet sent" << std::endl;
	collector.stop();
/*
	boost::filesystem::directory_iterator end_itr;
	std::string image_dir("../../images/snapshots_" + std::to_string(s));
	ImageSender image_sender(s, "http://pelars.sssup.it/pelars/", sm.getToken(), true);
	std::cout << "Sending images in " << image_dir << std::endl;
	for(boost::filesystem::directory_iterator itr(image_dir); itr != end_itr; ++itr){
		std::string path = itr->path().string();
		std::string name = itr->path().filename().string();
		std::cout << "sending " << path << std::endl;
	    if(boost::filesystem::is_regular_file(path)){
	    		std::ifstream in(path, std::ifstream::binary);
	    		in.unsetf(std::ios::skipws);
	    	    in.seekg(0, std::ios::end);
	    	    std::streampos fileSize = in.tellg();
	    	    in.seekg(0, std::ios::beg);
	    	    std::vector<char> data(fileSize);
	    		in.read(&data[0], fileSize);
	    		std::string code = base64_encode((unsigned char*)&data[0], (unsigned int)data.size());
	    		std::string type = path.substr(path.size() - 3, path.size());
	    		std::size_t start = name.find("_") + 1;
	    		std::string view = name.substr(0, start - 1);
	    		std::size_t end = name.substr(start, name.size()).find("_");
	    		long time_epoch = stol(name.substr(start, end));
	    		//std::cout << time_epoch << std::endl;
	    		//std::cout << "name " << view << " type " << type << std::endl;
	    		image_sender.send(code, type, view, true, time_epoch);
	    		in.close();
	    		sleep(1);
	    }
	}*/
	// Close websocket and session
	sm.closeSession(s, time);

	return 0;
}

