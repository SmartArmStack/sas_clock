#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
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

#include <atomic>
#include <chrono>

namespace sas
{

const int NSEC_TO_SEC = 1000000000;
const double NSEC_TO_SEC_D = 1000000000.0;

class Clock
{
private:

    std::chrono::system_clock::time_point time_initial_;
    std::chrono::system_clock::time_point next_loop_deadline_;

    std::chrono::system_clock::time_point time_before_sleep_;
    std::chrono::system_clock::time_point time_after_sleep_;

    std::chrono::nanoseconds target_sampling_time_;
    std::chrono::nanoseconds loop_duration_;
    std::chrono::nanoseconds computation_duration_;
    std::chrono::nanoseconds sleep_duration_;

    long overrun_sampling_time_count_;

    void _print_license_header();

public:
    Clock()=delete;

    explicit Clock(const int& thread_sampling_time_nsec);
    explicit Clock(const double& thread_sampling_time_nsec_d);

    void init();

    void update_and_sleep();

    std::chrono::system_clock::time_point get_initial_time() const;

    double get_sleep_time() const;

    std::chrono::system_clock::time_point get_last_update_time() const;

    double get_computation_time() const;

    double get_desired_thread_sampling_time_sec() const;

    double get_effective_thread_sampling_time_sec() const;

    double get_elapsed_time_sec() const;

    void safe_sleep_seconds(const double& seconds, std::atomic_bool* break_loop);

    void blocking_sleep_seconds(const double& seconds);

    long get_overrun_count() const;
};

}




