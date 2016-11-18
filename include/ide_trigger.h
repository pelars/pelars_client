#pragma once
#include "data_writer.h"
#include "trigger.h"

struct IdeTrigger
{

	IdeTrigger(ChannelWrapper<Trigger> & q) : queues_(q){}

	DataWriter * data_writer_;
	ChannelWrapper<Trigger> & queues_;

};