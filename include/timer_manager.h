/**
 * Project: CoCo
 * Copyright (c) 2016, Scuola Superiore Sant'Anna
 *
 * Authors: Filippo Brizzi <fi.brizzi@sssup.it>, Emanuele Ruffaldi
 * 
 * This file is subject to the terms and conditions defined in
 * file 'LICENSE.txt', which is part of this source code package.
 */

#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include <unordered_map>
#include <ctime>
#include <chrono>
#include <vector>
#include <mutex>
#include <cmath>
#include <limits>
#include <algorithm>
#include <atomic>

inline int long time()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
}

struct TimeStatistics
{
    unsigned long iterations;
    double last;
    double elapsed;
    double mean;
    double variance;
    double service_mean;
    double service_variance;
    double min;
    double max;
    char name[60];

    std::string toString() const
    {
        std::stringstream ss;
        //ss << "\tName: " << name << std::endl;
        ss << "\tIterations           : " << iterations << std::endl;
        ss << "\tRun Total (s)        : " << elapsed << std::endl;
        ss << "\tRun Mean             : " << mean << std::endl;
        ss << "\tRun Variance         : " << variance << std::endl;
        ss << "\tRun Min              : " << min << std::endl; 
        ss << "\tRun Max              : " << max << std::endl;
        ss << "\tService time mean    : " << service_mean << std::endl;
        ss << "\tService time variance: " << service_variance << std::endl;
        ss << "\tWait time mean       : " << service_mean-mean << std::endl;
        return ss.str();
    }
};

class Timer
{
public:
    using time_point = std::chrono::system_clock::time_point;

    explicit Timer(std::string timer_name = "")
        : name_(timer_name)
    {}

    Timer(const Timer &other)
        : name_(other.name_)
    {}

    void start()
    {
        while (lock_.exchange(true));

        auto now = std::chrono::system_clock::now();
        if (iterations_ != 0)
        {
            auto time = std::chrono::duration_cast<std::chrono::microseconds>(
                        now - start_time_).count() / 1000000.0;
            service_time_ += time;
            service_time_square_ += time * time;
        }

        start_time_ = now;
        ++iterations_;

        lock_ = false;
    }

    void stop()
    {
        while (lock_.exchange(true));
        

        time_ = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now() - start_time_).count() / 1000000.0;
        elapsed_time_ += time_;
        elapsed_time_square_ += time_ * time_;

        min_time_ = std::min(time_, min_time_);
        max_time_ = std::max(time_, max_time_);

        lock_ = false;
    }

    void reset()
    {
        while (lock_.exchange(true));

        iterations_ = 0;
        time_ = 0;
        elapsed_time_ = 0;
        elapsed_time_square_ = 0;
        service_time_ = 0;
        service_time_square_ = 0;
        max_time_ = 0;
        min_time_ = {std::numeric_limits<double>::infinity()};

        lock_ = false;
    }

    TimeStatistics timeStatistics()
    {
        TimeStatistics t;
        
        while (lock_.exchange(true));
        //std::cout << name_ << std::endl;
        //strcpy(t.name, name_.c_str());
        t.last = time_;
        t.iterations = iterations_;
        t.elapsed = elapsed_time_;
        t.mean = elapsed_time_ / iterations_;
        t.variance = (elapsed_time_square_ / iterations_) -
                        std::pow(elapsed_time_ / iterations_, 2);
        t.service_mean = service_time_ / (iterations_ - 1);
        t.service_variance = (service_time_square_ / (iterations_ - 1)) -
                                std::pow(service_time_ / (iterations_ - 1), 2);
        t.min = min_time_;
        t.max = max_time_;

        lock_ = false;

        return t;
    }

    double time()
    {
        while (lock_.exchange(true));
        auto t = time_;
        lock_ = false;
        return t;
    }

    double meanTime()
    {
        while (lock_.exchange(true));
        auto t = elapsed_time_ / iterations_;
        lock_ = false;
        return t;
    }

private:
    const std::string &name_;
    time_point start_time_;
    unsigned long iterations_ = 0;
    double time_ = 0;
    double elapsed_time_ = 0;
    double elapsed_time_square_ = 0;

    double service_time_ = 0;
    double service_time_square_ = 0;
    double max_time_ = 0;
    double min_time_ = {std::numeric_limits<double>::infinity()};

    std::atomic<bool> lock_ = {false};
};


class TimerManager
{
public:
    using time_point = std::chrono::system_clock::time_point;

    static TimerManager* instance()
    {
        static TimerManager timer_manager;
        return &timer_manager;
    }

    void startTimer(const std::string &name)
    {
        while (lock_.exchange(true));
        if (timer_list_.find(name) == timer_list_.end())
            timer_list_.insert({name, std::make_shared<Timer>(name)});
        timer_list_[name]->start();
        lock_ = false;
    }
    void stopTimer(const std::string &name)
    {
        while (lock_.exchange(true));
        auto t = timer_list_.find(name);
        if (t != timer_list_.end())
            t->second->stop();
        lock_ = false;
    }
    void removeTimer(const std::string &name)
    {
        while (lock_.exchange(true));
        auto t = timer_list_.find(name);
        if (t != timer_list_.end())
            timer_list_.erase(t);
        lock_ = false;
    }
    double time(const std::string &name)
    {
        while (lock_.exchange(true));
        auto t = timer_list_.find(name);
        if (t == timer_list_.end())
        {
            lock_ = false;
            return -1;
        }
        auto p = t->second;
        lock_ = false;
        return p->time();
    }
    double meanTime(const std::string &name)
    {
        while (lock_.exchange(true));
        auto t = timer_list_.find(name);
        if (t == timer_list_.end())
        {
            lock_ = false;
            return -1;
        }
        auto p = t->second;
        lock_ = false;
        return p->meanTime();
    }
    TimeStatistics timeStatistics(const std::string &name)
    {
        while (lock_.exchange(true));
        auto t = timer_list_.find(name);
        if (t == timer_list_.end())
        {
            lock_ = false;
            return TimeStatistics();
        }
        auto p = t->second;
        lock_ = false;
        return p->timeStatistics();
    }

    void resetTimers()
    {
        while (lock_.exchange(true))
            timer_list_.clear();
        lock_ = false;
    }


private:
    TimerManager()
    { }

    std::unordered_map<std::string, std::shared_ptr<Timer> > timer_list_;
    std::atomic<bool> lock_ = {false};
};

struct TimerScope
{
    TimerScope(TimerManager * x, const char * name) : tm_(x), name_(name) { x->startTimer(name); }
    ~TimerScope() { tm_->stopTimer(name_); }

    TimerManager * tm_;
    const char * name_;
};
