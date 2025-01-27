#include "Orientation.h"
#include <Wire.h>

Orientation::Orientation(uint8_t MPU) {
    this->MPU = MPU;
}

void Orientation::begin() {
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    wire.wrie(0x00);
    Wire.endTransmission(true);

    calculateOffsets(300);

    #ifdef DRONE_LOG
    consoleLog("Initiated orientation", true);
    #endif
}

void Orientation::begin(uint16_t cycles) {
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    wire.wrie(0x00);
    Wire.endTransmission(true);

    calculateOffsets(cycles);

    #ifdef DRONE_LOG
    consoleLog("Initiated orientation", true);
    #endif
}

void Orientation::end() {
    // TODO
}

void Orientation::readFromIMU(vector& acceleration, vector& angularVelocity) {
    // Acceleration
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    acceleration.x = (Wire.read() << 8 | Wire.read()) / 163840.0;
    acceleration.y = (Wire.read() << 8 | Wire.read()) / 163840.0;
    acceleration.z = (Wire.read() << 8 | Wire.read()) / 163840.0;

    // Gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    angularVelocity.x = (Wire.read() << 8 | Wire.read()) / 131.0;
    angularVelocity.y = (Wire.read() << 8 | Wire.read()) / 131.0;
    angularVelocity.z = (Wire.read() << 8 | Wire.read()) / 131.0;
}

void Orientation::readFromIMU(bool applyOffset) {
    readFromIMU(acceleration, angularVelocity);
    
    if (applyOffset) {
        acceleration += accelerationOffset;
        angularVelocity += angularVelocityOffset;
    }
}

vector Orientation::calculateAccelerationAngles(vector& acceleration) {
    return {(atan2(acceleration.y, sqrt(pow(acceleration.x, 2) + pow(acceleration.z, 2))) * 180 / PI)
            (atan2(-1 * acceleration.x, sqrt(pow(acceleration.y, 2) + pow(acceleration.z, 2))) * 180 / PI),
            0};
}

void Orientation::update(float deltaTime) {
    readFromIMU(acceleration, angularVelocity);

    calculateAngles(deltaTime);    
    calculateVelocity(deltaTime);
}

void Orientation::calculateAngles(float deltaTime) {
    vector accelerationAngles = calculateAccelerationAngles(acceleration) - accelerationAngleOffset;

    float rateOfChange = 0.1;
    angles += ((angularVelocity * deltaTime) + angleError) * rateOfChange;

    accelerationError += (accelerationAngles - angles) * deltaTime;
    angleError =  accelerationAngles + accelerationError - angles;
}

void Orientation::calculateVelocity(float deltaTime) {
    float sinPitch = sin(angles.x);
    float sinRoll = sin(angles.y);
    float cosPitch = cos(angles.x);
    float cosRoll = cos(angles.y);
    
    /*
        Acording to ChatGPT, this rotational matrix transforms the acceleration values to the world frame
        |cos(pitch)             0               -sin(pitch)     |
        |sin(roll)sin(pitch)    cos(roll)   sin(roll)cos(pitch) |
        |cos(roll)sin(pitch)    -sin(roll)  cos(roll)cos(pitch) |
        
        Looks reasonable but haven't checked
    */

    vector adjustedAcceleration = {
        acceleration.x * cosPitch + acceleration.z * -sinPitch,
        acceleration.x * sinRoll * sinPitch + acceleration.y * cosRoll + acceleration.z * sinRoll * cosPitch,
        acceleration.x * cosRoll * sinPitch + acceleration.y * -sinRoll + acceleration.z * cosRoll * cosPitch
    }

    velocity += adjustedAcceleration * deltaTime;
}

void Orientation::calculateOffsets(uint16_t cycles) {
    angularVelocityOffset = {0, 0, 0};
    accelerationOffset = {0, 0, 0};
    accelerationAngleOffset = {0, 0, 0};
    
    vector acceleration;
    vector angularVelocity;

    for (int i = 0; i < cycles; i++) {
        readFromIMU(acceleration, angularVelocity);

        angularVelocityOffset += angularVelocity;
        accelerationOffset += acceleration;
    }
    
    angularVelocityOffset /= cycles;
    accelerationOffset /= cycles;

    for (int i = 0; i < cycles; i++) {
        readFromIMU(acceleration, angularVelocity);

        accelerationAngleOffset += calculateAccelerationAngles(acceleration + accelerationOffset);
    }

    accelerationAngleOffset /= cycles;
}