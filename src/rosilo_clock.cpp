/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_clock.
#
#    rosilo_clock is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_clock is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_clock.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/


#include <ros/ros.h>
#include <thread>

#include "rosilo_clock/rosilo_clock.h"

namespace rosilo
{

void Clock::_print_license_header()
{
    ROS_WARN_STREAM("*********************************");
    ROS_WARN_STREAM("RTC LGPLv3 by Murilo@UT (c) 2020 ");
    ROS_WARN_STREAM("*********************************");
}

Clock::Clock(const int& thread_sampling_time_nsec)
{
    _print_license_header();
    target_sampling_time_ = std::chrono::nanoseconds(thread_sampling_time_nsec);
}

Clock::Clock(const double& thread_sampling_time_nsec_d)
{
    _print_license_header();
    target_sampling_time_ = std::chrono::nanoseconds(int(thread_sampling_time_nsec_d));
}

void Clock::init()
{
    // Get initial time
    time_initial_ = std::chrono::system_clock::now();

    // Initialize all time points
    time_before_sleep_ = time_initial_;
    time_after_sleep_ = time_initial_;
    next_loop_deadline_ = time_initial_;
}

void Clock::update_and_sleep()
{
    time_before_sleep_ = std::chrono::system_clock::now();
    // The required computation time
    computation_duration_ = time_before_sleep_ - time_after_sleep_;
    // Define the next deadline
    next_loop_deadline_ += target_sampling_time_;

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
    return (double(computation_duration_.count())/NSEC_TO_SEC_D);
}

double Clock::get_desired_thread_sampling_time_sec() const
{
    return (double(target_sampling_time_.count()/NSEC_TO_SEC_D));
}

double Clock::get_effective_thread_sampling_time_sec() const
{
    return (double((sleep_duration_ + computation_duration_).count())/NSEC_TO_SEC_D);
}

double Clock::get_elapsed_time_sec() const
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    return double(std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - get_initial_time()).count())/NSEC_TO_SEC_D;
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

};


