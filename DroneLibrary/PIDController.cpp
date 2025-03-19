#include "PIDController.h"

float PID::calculate(float inputValue, float deltaTime) {
    float error = *targetValue - inputValue;

    float derivitive = (error - previousError) / deltaTime;
    previousError = error;

    integral += error * deltaTime;

    return *p * error + *i * integral + *d * derivitive;
}

void PID::setTarget(float *targetValue) {
    this->targetValue = targetValue;
}

void PID::setConstants(const float *p, const float *i, const float *d) {
    this->p = p;
    this->i = i;
    this->d = d;
}

void PID::setConstants(const PID_Instructions& pid) {
    this->p = &pid.p;
    this->i = &pid.i;
    this->d = &pid.d;
}