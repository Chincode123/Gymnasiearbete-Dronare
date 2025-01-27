#include "MotorControl.h"
 
MotorController::MotorController(uint8_t& motorPowerTL, uint8_t& motorPowerTR, uint8_t& motorPowerBR, uint8_t& motorPowerBL)
 : motorPowerTL(motorPowerTL), motorPowerTR(motorPowerTR), motorPowerBR(motorPowerBR), motorPowerBL(motorPowerBL) {}

void MotorController::setTargetValues(float *targetVelocity, float *targetPitch, float *targetRoll) {
    velocityController.setTarget(targetVelocity);
    pitchController.setTarget(targetPitch);
    rollController.setTarget(targetRoll);
}

void MotorController::setVelocityConstants(float *p, float *i, float *d) {
    velocityController.setConstants(p, i, d);
}

void MotorController::setPitchConstants(float *p, float *i, float *d) {
    pitchController.setConstants(p, i, d);
}

void MotorController::setRollConstants(float *p, float *i, float *d) {
    rollController.setConstants(p, i, d);
}

void MotorController::calculatePower(float velocity, float pitch, float roll, float deltaTime) {
    float basePower = velocityController.calculate(velocity, deltaTime);
    basePower = constrain(basePower, 0, 255);
    float pitchShift = pitchController.calculate(pitch, deltaTime);
    float rollShift = rollController.calculate(roll, deltaTime);

    motorPowerTL = (uint8_t)constrain((basePower + pitchShift + rollShift), 0, 255);
    motorPowerTR = (uint8_t)constrain((basePower + pitchShift - rollShift), 0, 255);
    motorPowerBR = (uint8_t)constrain((basePower - pitchShift - rollShift), 0, 255);
    motorPowerBL = (uint8_t)constrain((basePower - pitchShift + rollShift), 0, 255);
}