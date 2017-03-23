#pragma once
#include <vector>
#include <memory>
#include "pooledchannel.hpp"
#include "image_frame.h"
#include "trigger.h"


template<typename T>
std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<T>>>> makeChannel(const size_t size, const bool & stopper, const size_t ch_size){

	std::vector<std::shared_ptr<PooledChannel<std::shared_ptr<T>>>> channels(size);

	for(int i=0; i < size; ++i){
		std::shared_ptr<PooledChannel<std::shared_ptr<T>>> ch = std::make_shared<PooledChannel<std::shared_ptr<T>>>(ch_size, true, false);
		ch->set_termination(&stopper);
		channels[i] = ch;
	}

	return channels;
}

