#ifndef TIMER_H_
#define TIMER_H_

#include <Arduino.h>

class CountdownTimer {
    long long startTime;
    long long timer;
public:
    CountdownTimer();
    void start(long long time);
    void stop();
    bool finished();
    bool finished(long long time);
};
#endif