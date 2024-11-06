#include "motorControl.h"
 
MotorController::MotorController(float& motorPower1, float& motorPower2, float& motorPower3, float& motorPower4) {
    this->motorPower1 = motorPower1;
    this->motorPower2 = motorPower2;
    this->motorPower3 = motorPower3;
    this->motorPower4 = motorPower4;
}

MotorController::setTargetValues(float& targetVelocity, float& targetPitch, float& targetRoll) {
    velocityController.setTarget(targetVelocity);
    pitchController.setTarget(targetPitch);
    rollController.setTarget(targetRoll);
}

MotorController::setVelocityConstants(float& p, float& i, float& d) {
    velocityController.setConstants(p, i, d);
}

MotorController::setPitchConstants(float& p, float& i, float& d) {
    pitchController.setConstants(p, i, d);
}

MotorController::setRollConstants(float& p, float& i, float& d) {
    rollController.setConstants(p, i, d);
}

MotorController::calculatePower(vector& rotationalSpeed, vector& acceleration, float& deltaTime) {
    // TODO: set velocity and rotation

    float basePower = velocityController.calculate(velocity.y, deltaTime);
    float pitchShift = pitchController.calculate(angles.x); // *not sure what axis gyroscope is aligned to
    float rollShift = rollController.calculate(angles.z);

    motorPower1 = basePower + pitchShift + rollShift;
    motorPower2 = basePower + pitchShift - rollShift;
    motorPower3 = basePower - pitchShift - rollShift;
    motorPower4 = basePower - pitchShift + rollShift;
}