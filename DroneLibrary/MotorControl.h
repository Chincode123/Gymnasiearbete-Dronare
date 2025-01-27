#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include "../utils/PIDController.h"

class MotorController {
public:    
    MotorController(uint8_t& motorPowerTL, uint8_t& motorPowerTR, uint8_t& motorPowerBR, uint8_t& motorPowerBL);

    void setTargetValues(float targetVelocity, float targetPitch, float targetRoll);
    void setVelocityConstants(float p, float i, float d);
    void setPitchConstants(float p, float i, float d);
    void setRollConstants(float p, float i, float d);

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