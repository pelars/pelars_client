#pragma once
#include <memory>
#include "pooledchannel.hpp"
#include "image_frame.h"

void show_markers(const unsigned int id, const float marker_size, std::shared_ptr<PooledChannel<std::shared_ptr<ImageFrame>>> pcw);