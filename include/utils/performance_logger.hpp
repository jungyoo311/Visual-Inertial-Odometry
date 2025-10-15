#ifndef PERFORMANCE_LOGGER
#define PERFORMANCE_LOGGER
#include <ctime>
#include <cstdlib>
/**
 * @brief Key Performance Indicator
 * 1. Number of Active Features
 * 2. Average Track Length
 *      // Prev(True) Curr(True): Green-dots
        // Prev(False) Curr(True): Blue-dots
        // Prev(True) Curr(False): Red-dots
 * 3. Number of New Features
 */
class Logger
{
    public:
        Logger();
        ~Logger();
    private:


};
#endif