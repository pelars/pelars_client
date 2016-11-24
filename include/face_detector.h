#pragma once

class DataWriter;

extern bool to_stop;
extern bool visualization;
extern double interval;

void detectFaces(DataWriter & websocket, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pcw, const bool video);
