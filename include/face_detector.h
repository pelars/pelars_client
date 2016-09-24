#pragma once


class DataWriter;
class ScreenGrabber;
class ImageSender;

extern bool to_stop;
extern bool visualization;
extern double interval;
extern bool snapshot_people;
extern bool snapshot_screen;

void detectFaces(DataWriter & websocket, ScreenGrabber & screen_grabber, 
	             ImageSender & image_sender_screen, ImageSender & image_sender_people, 
	             const int face_camera_id, const bool video);
