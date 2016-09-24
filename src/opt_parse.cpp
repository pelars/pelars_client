#include "opt_parse.h"

Parser::Parser(int argc, char ** argv):argc_(argc), argv_(argv), description_("Pelars Client Usage")
{
    description_.add_options()
            ("help", "help message")
            ("face,f", "track the faces")
            ("mongoose,m", "mongoose port for the arduino ide")
            ("audio,a", "track audio level")
            ("marker,M",  boost::program_options::value<float>(), "marker size")
            ("hand,h", "track the hands")
            ("particle,p", "track the particle.io sensors")
            ("ide,i", "track the Arduino IDE log")
            ("marker_id,I", "shows the marker id on screen")
            ("video,V", "stores the video stream")
            ("visualization,v", "activate visualization")
            //("object,O", boost::program_options::value<std::string>(), "object template file")
            ("qr,q", "show session as qr code")
            ("server,S", boost::program_options::value<std::string>(), "server endpoint")
            ("session,x", boost::program_options::value<int>(), "session id")
            ("face_camera,F", boost::program_options::value<int>(), "video device id (ex: for /dev/video3 use -f 3)")
            ("calibration,c", "calibrate cameras")
            ("no_webcam_calib,w", "don't use webcam calibration")
            ("no_kinect2_calib,k", "con't use kinect2 claibration")
            ("default,D", "create a session with default flags (-f -h -a -i -j -q")
            ("special,s", "special flag for background run")
            ("processor,X", boost::program_options::value<int>(), "kinect2 processor : 0 for CPU, 1 for OPENCL, 2 for OPENGL. default 1")
            ("upload,u", boost::program_options::value<std::string>(), "file name to upload")
            ("status,j", "displays system status");

    boost::program_options::options_description hidden("Hidden options");
    boost::program_options::store(boost::program_options::command_line_parser(argc_, argv_).options(description_).run(), vm_);
    boost::program_options::notify(vm_);
}

bool Parser::get(std::string value){
    if(vm_.count(value))
        return true;
    return false;
}

void Parser::printHelp(){
    std::cout << description_ << std::endl;
}

std::string Parser::getString(std::string value){
    if(vm_.count(value))
        return vm_[value].as<std::string>();
    throw std::runtime_error(std::string("Parsed argument is not a string"));
}

float Parser::getFloat(std::string value){
    if(vm_.count(value))
        return vm_[value].as<float>();
    throw std::runtime_error(std::string("Parsed argument is not a float"));
}

int Parser::getInt(std::string value){
    if(vm_.count(value))
        return vm_[value].as<int>();
    throw new std::exception;
}

