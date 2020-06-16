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

#include "rosilo_clock/rosilo_clock.h"

namespace rosilo
{

//Auxiliar function
void Clock::timespec_diff(const struct timespec& start, const struct timespec& stop,
                          struct timespec& result) const
{
    if ((stop.tv_nsec - start.tv_nsec) < 0)
    {
        result.tv_sec = stop.tv_sec - start.tv_sec - 1;
        result.tv_nsec = stop.tv_nsec - start.tv_nsec + 1000000000;
    }
    else
    {
        result.tv_sec = stop.tv_sec - start.tv_sec;
        result.tv_nsec = stop.tv_nsec - start.tv_nsec;
    }

    return;
}

double Clock::timespec_to_double_sec(const struct timespec& ts) const
{
    return double(ts.tv_sec)+(double(ts.tv_nsec)/NSEC_TO_SEC_D);
}




Clock::Clock(const int& thread_sampling_time_nsec)
{
    ROS_WARN_STREAM("*********************************");
    ROS_WARN_STREAM("RTC LGPLv3 by Murilo@UT (c) 2020 ");
    ROS_WARN_STREAM("*********************************");
    thread_sampling_time_nsec_   = thread_sampling_time_nsec;
    thread_sampling_time_nsec_d_ = (double)thread_sampling_time_nsec_;
    thread_sampling_time_sec_d_  = thread_sampling_time_nsec_d_/NSEC_TO_SEC_D;
}

Clock::Clock(const double& thread_sampling_time_nsec_d)
{
    ROS_WARN_STREAM("*********************************");
    ROS_WARN_STREAM("RTC LGPLv3 by Murilo@UT (c) 2020 ");
    ROS_WARN_STREAM("*********************************");
    thread_sampling_time_nsec_d_ = thread_sampling_time_nsec_d;
    thread_sampling_time_nsec_   = int(thread_sampling_time_nsec_d);
    thread_sampling_time_sec_d_  = thread_sampling_time_nsec_d_/NSEC_TO_SEC_D;
}

void Clock::init()
{
    clock_gettime(CLOCK_MONOTONIC,&loop_time_);
    clock_gettime(CLOCK_MONOTONIC,&after_loop_);
    clock_gettime(CLOCK_MONOTONIC,&before_loop_);
    initial_time_ = double(loop_time_.tv_sec)+(double(loop_time_.tv_nsec)/NSEC_TO_SEC_D);
    sleep_time_d_ = 0;
    computation_time_d_ = 0;
    after_loop_d_ = 0;
}

void Clock::update_and_sleep()
{
    clock_gettime(CLOCK_MONOTONIC,&before_loop_);
    timespec_diff(after_loop_,before_loop_,computation_time_);
    computation_time_d_ = (computation_time_.tv_sec)+(double(computation_time_.tv_nsec)/NSEC_TO_SEC_D);
    //Update clock and sleep
    if (loop_time_.tv_nsec + thread_sampling_time_nsec_ >= NSEC_TO_SEC)
    {
        loop_time_.tv_nsec = loop_time_.tv_nsec - NSEC_TO_SEC + thread_sampling_time_nsec_;
        loop_time_.tv_sec  = loop_time_.tv_sec  + 1;
    }
    else
    {
        loop_time_.tv_nsec += thread_sampling_time_nsec_;
    }
    clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&loop_time_,NULL);
    clock_gettime(CLOCK_MONOTONIC,&after_loop_);
    after_loop_d_ = double(after_loop_.tv_sec)+(double(after_loop_.tv_nsec)/NSEC_TO_SEC_D);
    timespec_diff(before_loop_,after_loop_,elapsed_time_);
    sleep_time_d_ = double(elapsed_time_.tv_sec)+(double(elapsed_time_.tv_nsec)/NSEC_TO_SEC_D);
}

double Clock::get_initial_time() const
{
    return initial_time_;
}

double Clock::get_sleep_time() const
{
    return sleep_time_d_;
}

double Clock::get_last_update_time() const
{
    return after_loop_d_;
}

double Clock::get_computation_time() const
{
    return computation_time_d_;
}

double Clock::get_desired_thread_sampling_time_sec() const
{
    return thread_sampling_time_sec_d_;
}

double Clock::get_effective_thread_sampling_time_sec() const
{
    return computation_time_d_+sleep_time_d_;
}

double Clock::get_elapsed_time_sec() const
{
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC,&current_time);
    return (timespec_to_double_sec(current_time)-get_initial_time());
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


