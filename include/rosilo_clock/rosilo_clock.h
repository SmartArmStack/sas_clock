#pragma once

/*
# Copyright (c) 2016
# Mitsuishi Sugita Laboratory (NML) at University of Tokyo
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

#include <atomic>

namespace rosilo
{

const int NSEC_TO_SEC = 1000000000;
const double NSEC_TO_SEC_D = 1000000000.0;

class Clock
{
private:

    struct timespec loop_time_;
    struct timespec before_loop_;
    struct timespec after_loop_;
    struct timespec elapsed_time_;
    struct timespec computation_time_;
    long   thread_sampling_time_nsec_;
    double thread_sampling_time_nsec_d_;
    double thread_sampling_time_sec_d_;
    double initial_time_;

    double sleep_time_d_;
    double computation_time_d_;
    double after_loop_d_;

    //Auxiliar function
    void timespec_diff(const struct timespec& start, const struct timespec& stop,
                       struct timespec& result) const;

    double timespec_to_double_sec(const struct timespec& ts) const;

public:
    Clock()=delete;

    explicit Clock(const int& thread_sampling_time_nsec);

    explicit Clock(const double& thread_sampling_time_nsec_d);

    void init();

    void update_and_sleep();

    double get_initial_time() const;

    double get_sleep_time() const;

    double get_last_update_time() const;

    double get_computation_time() const;

    double get_desired_thread_sampling_time_sec() const;

    double get_effective_thread_sampling_time_sec() const;

    double get_elapsed_time_sec() const;

    void safe_sleep_seconds(const double& seconds, std::atomic_bool* break_loop);

    void blocking_sleep_seconds(const double& seconds);

};

}




