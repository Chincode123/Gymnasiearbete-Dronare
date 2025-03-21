#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <Arduino.h>
#include "PIDController.h"
#include "RadioData.h"

class MotorController {
public:    
    MotorController(int8_t& motorPowerTL, int8_t& motorPowerTR, int8_t& motorPowerBR, int8_t& motorPowerBL);

    void setTargetValues(float *targetVelocity, float *targetPitch, float *targetRoll);
    void setVelocityConstants(const PID_Instructions &values);
    void setPitchConstants(const PID_Instructions &values);
    void setRollConstants(const PID_Instructions &values);

    void calculatePower(float velocity, float pitch, float roll, float deltaTime);
private:
    int8_t &motorPowerTL, &motorPowerTR, &motorPowerBR, &motorPowerBL;

    // PID controller for the vertical velocity of the drone
    PID velocityController;
    // PID controller for the pitch of the drone
    PID pitchController;
    // PID controller for the roll of the drone
    PID rollController;
};
#endif