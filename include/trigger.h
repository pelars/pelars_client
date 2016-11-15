#pragma once
#include <chrono>
#include <vector>
#include <pooledchannel.hpp>
#include <memory>
#include <unistd.h>

extern bool to_stop;

class Trigger{

public:

	Trigger(std::chrono::high_resolution_clock::time_point pt, bool automatic): time_(pt), automatic_(automatic) {}
	Trigger() {}

	std::chrono::high_resolution_clock::time_point time_;
	bool automatic_;

};

void sendTrigger(const std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>> & pc_trigger, int interval);