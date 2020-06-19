/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
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

    // Set next loop's timeline
    next_loop_deadline_ = time_initial_ + target_sampling_time_;
}

void Clock::update_and_sleep()
{
    time_before_sleep_ = std::chrono::system_clock::now();
    // The required computation time
    computation_duration_ = time_before_sleep_ - time_after_sleep_;
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
    return computation_duration_.count();
}

double Clock::get_desired_thread_sampling_time_sec() const
{
    return target_sampling_time_.count();
}

double Clock::get_effective_thread_sampling_time_sec() const
{
    return (sleep_duration_ + computation_duration_).count();
}

double Clock::get_elapsed_time_sec() const
{
    std::chrono::system_clock::time_point current_time = std::chrono::system_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(current_time - get_initial_time()).count();
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


