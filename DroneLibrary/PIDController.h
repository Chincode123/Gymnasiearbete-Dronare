#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <Arduino.h>

class PID {
    const float *p, *i, *d;
    float *targetValue;
    float previousError;
    float integral;
public:
    float calculate(float inputValue, float deltaTime);
    void setTarget(float *targetValue);
    void setConstants(const float *p, const float *i, const float *d);
};

#endif