#ifndef TIMER_H
#define TIMER_H
 
#include <chrono>

class TimeIt
{
public:
    TimeIt()
    {
        begin = std::chrono::steady_clock::now();
    }
    void from()
    {
        begin = std::chrono::steady_clock::now();
    }
    float now()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin).count();
    }
private:
    std::chrono::steady_clock::time_point begin;
};

#endif