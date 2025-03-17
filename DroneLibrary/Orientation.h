#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include "Vectors.h"
#include "SmoothValue.h"

class Orientation {
    uint8_t MPU;

    vector3<float> angularVelocityOffset;
    vector3<float> accelerationOffset;
    vector3<float> accelerationAngleOffset;
    
    vector3<float> rawAngularVelocity;
    vector3<float> previousAccelerationAngles;
    vector3<SmoothValue> anglularVelocityError;

    void readFromIMU(vector3<float>& acceleration, vector3<float>& angularVelocity);
    void readFromIMU();

    void calculateOffsets(uint16_t cycles);

    vector3<float> calculateAccelerationAngles(const vector3<float>& acceleration);

    void calculateAngles(float deltaTime);
    void calculateVelocity(float deltaTime);

    float limitAngle(float angle);
public:
    Orientation(uint8_t MPU);

    void begin();
    void begin(uint16_t errorCycles);
    void end();

    vector3<float> angularVelocity;
    vector3<SmoothValue> angles;

    vector3<float> acceleration;
    vector3<float> adjustedAcceleration;
    vector3<SmoothValue> velocity;

    void update(float deltaTime);
};
#endif