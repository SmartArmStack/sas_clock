/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
#
#    This file is part of sas_clock.
#
#    sas_clock is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_clock is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_clock.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <iostream>
#include <thread>

#include "sas_clock/sas_clock.hpp"

namespace sas
{

void Clock::_print_license_header()
{
    //Using cout instead of depending on ROS 2 for now.
    std::cout << "******************************************" << std::endl;
    std::cout << "sas::Clock LGPLv3 by Murilo (c) 2016-2023"  << std::endl;
    std::cout << "******************************************" << std::endl;
}

Clock::Clock(const double& sampling_time_in_seconds):
    overrun_sampling_time_count_(0)
{
    _print_license_header();
    target_sampling_time_ = std::chrono::nanoseconds(int(sampling_time_in_seconds*1e9));
}

void Clock::init()
{
    // Get initial time
    time_initial_ = std::chrono::system_clock::now();

    // Initialize all time points
    time_before_sleep_ = time_initial_;
    time_after_sleep_ = time_initial_;
    next_loop_deadline_ = time_initial_;

    overrun_sampling_time_count_ = 0;
}

void Clock::update_and_sleep()
{
    time_before_sleep_ = std::chrono::system_clock::now();

    // The required computation time
    computation_duration_ = time_before_sleep_ - time_after_sleep_;

    // Define the next deadline
    // It is important to not define an impossible deadline. We also add the number
    // of deadline overruns safely to a counter.
    next_loop_deadline_ += target_sampling_time_;
    while(next_loop_deadline_ < (time_before_sleep_))
    {
        next_loop_deadline_ += target_sampling_time_;
        // Increase counter only if it won't cause an overflow
        if(overrun_sampling_time_count_+1 < std::numeric_limits<long>().max())
        {
            overrun_sampling_time_count_++;
        }
    }

    //Sleep until the deadline
    std::this_thread::sleep_until(next_loop_deadline_);

    time_after_sleep_  = std::chrono::system_clock::now();
    // The time spend sleeping
    sleep_duration_ = time_after_sleep_ - time_before_sleep_;
}

std::chrono::system_clock::time_point Clock::get_initial_time() const
{
    return time_initial_;
}

double Clock::get_sleep_time() const
{
    return sleep_duration_.count();
}

std::chrono::system_clock::time_point Clock::get_last_update_time() const
{
    return time_after_sleep_;
}

double Clock::get_computation_time() const
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(computation_duration_).count();
}

double Clock::get_desired_thread_sampling_time_sec() const
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(target_sampling_time_).count();
}

double Clock::get_effective_thread_sampling_time_sec() const
{
    return std::chrono::duration_cast<std::chrono::duration<double>>(sleep_duration_ + computation_duration_).count();
}

double Clock::get_elapsed_time_sec() const
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::duration<double>>(current_time - get_initial_time()).count();
}

void Clock::safe_sleep_seconds(const double& seconds, std::atomic_bool* break_loop)
{
    for(int i=0;i<int(seconds/get_desired_thread_sampling_time_sec()) && not (*break_loop);i++)
    {
        update_and_sleep();
    }
}

void Clock::blocking_sleep_seconds(const double& seconds)
{
    for(int i=0;i<int(seconds/get_desired_thread_sampling_time_sec());i++)
    {
        update_and_sleep();
    }
}

long Clock::get_overrun_count() const
{
    return overrun_sampling_time_count_;
}

};


