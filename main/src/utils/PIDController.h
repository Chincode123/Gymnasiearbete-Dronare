#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

class PID {
    float p, i, d;
    float targetValue;
    float previousError;
    float integral;
public:
    float calculate(float inputValue, float deltaTime);
    void setTarget(float targetValue);
    void setConstants(float p, float i, float d);
};

#endif