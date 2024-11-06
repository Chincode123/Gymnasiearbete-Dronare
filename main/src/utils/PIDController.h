#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

class PID {
    PID(float& initialTarget, float& p, float& i, float& d);

    float calculate(float& inputValue, double& deltaTime);

    void setTarget(float& targetValue);

    void setConstants(float& p, float& i, float& d);
private:
    float p, i, d;
    float& targetValue;
    float previousError;
    float integral;
};

#endif