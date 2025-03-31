#include "Timer.h"

void Timer::start(long long time) {
    startTime = millis();
    timer = time;
}

void Timer::stop() {
    timer = 0;
}

bool Timer::finished() {
    if (timer == 0) {
        return false;
    }
    long long currentTime = millis();
    if (currentTime - startTime >= timer) {
        stop();
        return true;
    }
    return false;
}