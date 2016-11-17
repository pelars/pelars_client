#pragma once
#include "data_writer.h"
#include "trigger.h"

struct IdeTrigger
{

	IdeTrigger(std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>> & q) : queues_(q){}

	DataWriter * data_writer_;
	std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<Trigger>>>>  & queues_;

};