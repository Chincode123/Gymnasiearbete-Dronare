#include "CountdownTimer.h"

CountdownTimer::CountdownTimer() : 
    startTime(0), 
    timer(0) 
    {}

void CountdownTimer::start(long long time) {
    startTime = millis();
    timer = time;
}

void CountdownTimer::stop() {
    startTime = 0;
    timer = 0;
}

bool CountdownTimer::finished() {
    if (startTime == 0)
        return false;
    
    long long currentTime = millis();
    if (currentTime - startTime >= timer)
        return true;
    return false;
}

bool CountdownTimer::finished(long long time) {
    if (!finished())
        return false;
    start(time);
    return true;
}