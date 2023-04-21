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


#include <iostream>
#include <thread>
#include <chrono>

#include <sas_clock/sas_clock.hpp>

int main(int, char**)
{
    //10 ms
    sas::Clock clock(0.01);

    clock.init();
    for(int i=0;i<50;i++)
    {
        clock.update_and_sleep();
        std::cout << "Loop n = " << i << std::endl;
        std::cout << "  Elapsed time: " << clock.get_elapsed_time_sec() << std::endl;
        std::cout << "  Computation time: " << clock.get_computation_time() << std::endl;
        std::cout << "  Effective thread sampling time: " << clock.get_effective_thread_sampling_time_sec() << std::endl;
        std::cout << "  Desired thread sampling time: " << clock.get_desired_thread_sampling_time_sec() << std::endl;
        std::cout << "  Overrun count: " << clock.get_overrun_count() << std::endl << std::endl;
    }

    return 0;
}
