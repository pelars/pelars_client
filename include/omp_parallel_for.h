#pragma once
#include "threadpool.h"

inline void parallelFor(const unsigned int size, std::function<void(const unsigned int)> func)
{
	const unsigned int nbThreads = std::thread::hardware_concurrency();
	std::vector <std::thread> threads;
	for(unsigned int idThread = 0; idThread < nbThreads; ++idThread) {
		auto threadFunc = [=, &threads](){
			for(unsigned int i = idThread; i < size; i += nbThreads) {
				func(i);
			}
		};
		threads.push_back(std::thread(threadFunc));
	}
	for(auto & t : threads) 
		t.join();
}

inline void parallelFor(ThreadPool & pool, const unsigned int size, std::function<void(const unsigned int)> func)
{
	std::vector< std::future<void>> results;   
	results.reserve(size);
	for(unsigned int i=0; i < size; ++i) 
	{
		results.emplace_back(pool.enqueue([i, &func] {func(i);}));
	}
	for(auto && result: results)
		result.get();
}
