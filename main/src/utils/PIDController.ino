#include "PIDController.h"

PID::PID(float& initialTarget, float& p, float& i, float& d) {
       setTarget(targetValue);
       setConstants(p, i, d);
}

float PID::calculate(float& inputValue, float& deltaTime) {
    float error = targetValue - inputValue;

    float derivitive = (error - previousError) / deltaTime;
    previousError = error;

    integral += error * deltaTime;

    return p * error + i * integral + d * derivitive;
}

void PID::setTarget(float& targetValue) {
    this->targetValue = targetValue;
}

void PID::setConstants(float& p, float& i, float& d) {
    this->p = p;
    this->i = i;
    this->d = d;
}