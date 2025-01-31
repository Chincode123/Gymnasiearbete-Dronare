#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include "Vectors.h"
#include <Arduino.h>

class Orientation {
    uint8_t MPU;

    vector angularVelocityOffset;
    vector accelerationOffset;
    vector accelerationAngleOffset;
    
    vector angleError;
    vector accelerationError;

    void readFromIMU(vector& acceleration, vector& angularVelocity);
    void readFromIMU();

    void calculateOffsets(uint16_t cycles);

    vector calculateAccelerationAngles(const vector& acceleration);

    void calculateAngles(float deltaTime);
    void calculateVelocity(float deltaTime);
public:
    Orientation(uint8_t MPU);

    void begin();
    void begin(uint16_t errorCycles);
    void end();

    vector angularVelocity;
    vector angles;

    vector acceleration;
    vector adjustedAcceleration;
    vector velocity;

    void update(float deltaTime);
};
#endif