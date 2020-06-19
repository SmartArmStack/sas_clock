#include <iostream>

#include <rosilo_clock/rosilo_clock.h>

int main(int, char**)
{
    //10 ms
    rosilo::Clock clock(10000000);

    clock.init();
    for(int i=0;i<50;i++)
    {
        clock.update_and_sleep();
        std::cout << "Loop n = " << i << std::endl;
        std::cout << "  Elapsed time: " << clock.get_elapsed_time_sec() << std::endl;
        std::cout << "  Computation time: " << clock.get_computation_time() << std::endl;
        std::cout << "  Effective thread sampling time: " << clock.get_effective_thread_sampling_time_sec() << std::endl;
        std::cout << "  Desired thread sampling time: " << clock.get_desired_thread_sampling_time_sec() << std::endl << std::endl;
    }

    return 0;
}
