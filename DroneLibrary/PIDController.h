#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include <Arduino.h>
#include "RadioData.h"

class PID {
    const float *p, *i, *d;
    float *targetValue;
    float previousError;
    float integral;
public:
    float calculate(float inputValue, float deltaTime);
    void setTarget(float *targetValue);
    void setConstants(const float *p, const float *i, const float *d);
    void setConstants(const PID_Instructions& pid);
};

#endif