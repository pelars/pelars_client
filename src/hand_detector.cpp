#include "hand_detector.h"

void handDetector(KinectManagerExchange & kme, DataWriter & websocket, cv::VideoCapture & capture, int session){

	// OpenCV matrixes to contain the data acquired from the kinect
	cv::Mat color, depth;

	aruco::MarkerDetector MDetector;
    vector<aruco::Marker> markers;
    sleep(1); //else opencv crash for parallel windows instantiation
    cv::namedWindow("hands");

    Json::Value root;
    Json::StyledWriter writer;
    float x, y, z;
    bool kinect = false;

	while(!to_stop){
		//kinect
		/*
		if(!kme.get(color, depth)){
	      std::cout << "failed to fetch data from the kinect\n";
	      to_stop = true;
	      break;
	    }*/
	   	capture >> color;
	    //MDetector.detect(color, markers, cameraMatrix_, distCoeffs_, 0.035, true);

	    //ADD KINECT PARAMETERS TO GET WORLD COORDINATES
	    MDetector.detect(color, markers);
	    if(markers.size() > 0){
   			for(int i = 0; i < markers.size(); ++i)
   				if(markers[i].id != 1023){
   				// Get marker position
				markers[i].draw(color, cv::Scalar(0, 0, 255), 2);
				x = markers[i][0].x;
				y = markers[i][0].y;

				kinect ? z = depth.at<short>((int)y, (int)x) : 0.0f;
			
				// Prepare the json message
				root["obj"]["type"] = "hand";
		        root["obj"]["id"] = markers[i].id;
		        root["obj"]["x"] = x;
		        root["obj"]["y"] = y;
		        root["obj"]["z"] = z/1000;
		        root["obj"]["time"] = deltats(orwl_gettime(), start);
		        std::string out_string = writer.write(root);
		        //std::cout << "sending " << out_string << std::endl;
		        // Send the message online and store it offline
		        if(online)
		          io.post( [&websocket, out_string]() {
		               websocket.writeData(out_string);
		           });

		        websocket.writeLocal(out_string);
   			}	
    	}
    	// Display the image
    	int c = cv::waitKey(1);
		if((char)c == 'q' ) {
			to_stop = true;
			std::cout << "stop requested by hand detector" << std::endl;
		}
    	cv::imshow("hands", color);
	}
}

    