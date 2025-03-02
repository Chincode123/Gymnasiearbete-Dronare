#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "PIDController.h"
#include "RadioData.h"
#include <Arduino.h>

class MotorController {
public:    
    MotorController(uint8_t& motorPowerTL, uint8_t& motorPowerTR, uint8_t& motorPowerBR, uint8_t& motorPowerBL);

    void setTargetValues(float *targetVelocity, float *targetPitch, float *targetRoll);
    void setVelocityConstants(const PID_Instructions &values);
    void setPitchConstants(const PID_Instructions &values);
    void setRollConstants(const PID_Instructions &values);

    void calculatePower(float velocity, float pitch, float roll, float deltaTime);
private:
    uint8_t &motorPowerTL, &motorPowerTR, &motorPowerBR, &motorPowerBL;

    // PID controller for the vertical velocity of the drone
    PID velocityController;
    // PID controller for the pitch of the drone
    PID pitchController;
    // PID controller for the roll of the drone
    PID rollController;
};

#endif