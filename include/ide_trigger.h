#pragma once
#include "data_writer.h"
#include "trigger.h"

struct IdeTrigger
{

	IdeTrigger(ChannelWrapper<Trigger> & q, DataWriter * data_writer) : queues_(q), data_writer_(data_writer){}

	ChannelWrapper<Trigger> & queues_;
	DataWriter * data_writer_;
};