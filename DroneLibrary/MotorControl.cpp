#include "MotorControl.h"
 
MotorController::MotorController(uint8_t& motorPowerTL, uint8_t& motorPowerTR, uint8_t& motorPowerBR, uint8_t& motorPowerBL)
 : motorPowerTL(motorPowerTL), motorPowerTR(motorPowerTR), motorPowerBR(motorPowerBR), motorPowerBL(motorPowerBL) {}

void MotorController::setTargetValues(float *targetVelocity, float *targetPitch, float *targetRoll) {
    velocityController.setTarget(targetVelocity);
    pitchController.setTarget(targetPitch);
    rollController.setTarget(targetRoll);
}

void MotorController::setVelocityConstants(const PID_Instructions &values) {
    velocityController.setConstants(&values.k_p, &values.k_i, &values.k_d);
}

void MotorController::setPitchConstants(const PID_Instructions &values) {
    pitchController.setConstants(&values.k_p, &values.k_i, &values.k_d);
}

void MotorController::setRollConstants(const PID_Instructions &values) {
    rollController.setConstants(&values.k_p, &values.k_i, &values.k_d);
}

void MotorController::calculatePower(float velocity, float pitch, float roll, float deltaTime) {
    float basePower = velocityController.calculate(velocity, deltaTime);
    basePower = constrain(basePower, -127, 127);
    float pitchShift = pitchController.calculate(pitch, deltaTime);
    float rollShift = rollController.calculate(roll, deltaTime);

    motorPowerTL = (uint8_t)(constrain((basePower + pitchShift + rollShift), -127, 127) + 127);
    motorPowerTR = (uint8_t)(constrain((basePower + pitchShift - rollShift), -127, 127) + 127);
    motorPowerBR = (uint8_t)(constrain((basePower - pitchShift - rollShift), -127, 127) + 127);
    motorPowerBL = (uint8_t)(constrain((basePower - pitchShift + rollShift), -127, 127) + 127);
}