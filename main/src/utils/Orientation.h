#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include "Vectors.h"

class Orientation {
    uint8_t MPU;

    vector angularVelocityOffset;
    vector accelerationOffset;
    vector accelerationAngleOffset;
    
    vector angleError;
    vector accelerationError;

    void readFromIMU(vector& acceleration, vector& angularVelocity);
    void readFromIMU(bool applyOffsets);

    void calculateOffsets(uint16_t cycles);

    vector calculateAccelerationAngles(vector& acceleration);

    void calculateAngles(float deltaTime);
    void calculateVerticalVelocity(float deltaTime);
public:
    Orientation(uint8_t MPU);

    void begin();
    void begin(uint16_t errorCycles);
    void end();

    vector angularVelocity;
    vector angles;

    vector acceleration;
    float verticalVelocity;

    void update(float deltaTime);
};
#endif