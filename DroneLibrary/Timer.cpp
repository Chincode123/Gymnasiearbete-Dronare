#include "Timer.h"

Timer::Timer() : 
    startTime(0), 
    timer(0) 
    { }

void Timer::start(long long time) {
    startTime = millis();
    timer = time;
}

void Timer::stop() {
    startTime = 0;
    timer = 0;
}

bool Timer::finished() {
    if (startTime == 0)
        return false;
    
    long long currentTime = millis();
    if (currentTime - startTime >= timer)
        return true;
    return false;
}

bool Timer::finished(long long time) {
    if (!finished())
        return false;
    start(time);
    return true;
}