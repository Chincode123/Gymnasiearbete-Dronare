#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include <Arduino.h>
#include "Vectors.h"

class Orientation {
    const uint8_t MPU;

    vector3<float> angularVelocityOffset;
    vector3<float> accelerationOffset;
    vector3<float> accelerationAngleOffset;
    
    vector3<float> previousAccelerationAngles;
    vector3<float> angularVelocityError;

    void readFromIMU(vector3<float>& acceleration, vector3<float>& angularVelocity);
    void readFromIMU();

    void calculateOffsets(uint16_t cycles);

    vector3<float> calculateAccelerationAngles(const vector3<float>& acceleration);

    void calculateAngles(float deltaTime);
    void calculateVelocity(float deltaTime);

    float limitAngle(float angle);

    vector3<float> r;

    void correctAccelerationLever(float deltaTime);
public:
    Orientation(uint8_t MPUAddress, vector3<float> positionOffset);

    void begin();
    void begin(uint16_t errorCycles);
    void end();

    vector3<float> prevAngularVelocity;
    vector3<float> rawAngularVelocity;
    vector3<float> angularVelocity;
    vector3<float> angles;
    
    vector3<float> rawAcceleration;
    vector3<float> acceleration;
    vector3<float> adjustedAcceleration;
    vector3<float> velocity;

    void update(float deltaTime);
};
#endif