#ifndef MEASURE_TIME
#define MEASURE_TIME

#include <ctime>
#include <cstdlib>
#include <chrono>

class StopWatch
{
    public:
        StopWatch()
        {

        }
        void start_time()
        {
            start = std::chrono::system_clock::now();
        }
        double end_time()
        {
            end = std::chrono::system_clock::now();
            std::chrono::duration<double> elasped_seconds = end - start;
            return elasped_seconds.count() * 1000; // milliseconds
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> start, end;
};

#endif