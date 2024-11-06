#ifndef MOTORS_H_
#define MOTORS_H_

#include "../utils/PIDController.h"

class MotorController {
    MotorController(float& motorPower1, float& motorPower2, float& motorPower3, float& motorPower4);

    void setTargetValues(float& targetVelocity, float& targetPitch, float& targetRoll);
    void setVelocityConstants(float& p, float& i, float& d);
    void setPitchConstants(float& p, float& i, float& d);
    void setRollConstants(float& p, float& i, float& d);

    void calculatePower(float& velocity, float& pitch, float& roll, double& deltaTime);
private:
    float &motorPower1, &motorPower2, &motorPower3, &motorPower4;

    // PID controller for the vertical velocity of the drone
    PID velocityController(0, 0, 0, 0);
    // PID controller for the pitch of the drone
    PID pitchController(0, 0, 0, 0);
    // PID controller for the roll of the drone
    PID rollController(0, 0, 0, 0);
};

#endif