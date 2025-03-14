#include "Orientation.h"
#include <Wire.h>
#include <Arduino.h>

Orientation::Orientation(uint8_t MPU) {
    this->MPU = MPU;
    angles({
        SmoothValue(0, 0.1),
        SmoothValue(0, 0.1),
        SmoothValue(0, 0.1)
    });
    velocity({
        SmoothValue(0, 0.1),
        SmoothValue(0, 0.1),
        SmoothValue(0, 0.1)
    });
}

void Orientation::begin() {
    begin(300);
}

void Orientation::begin(uint16_t cycles) {
    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    calculateOffsets(cycles);
}

void Orientation::end() {
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(1 << 6);
    Wire.endTransmission(true);

    angularVelocityOffset = {0, 0 ,0};
    accelerationOffset = {0, 0 ,0};
    accelerationAngleOffset = {0, 0 ,0};
    angleError = {0, 0 ,0};
    accelerationError = {0, 0 ,0};
    angularVelocity = {0, 0 ,0};
    angles = {0, 0 ,0};
    acceleration = {0, 0 ,0};
    velocity = {0, 0 ,0};
}

void Orientation::readFromIMU(vector3<float>& acceleration, vector3<float>& angularVelocity) {
    // Acceleration
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    // values are between -2(g) and 2(g)
    acceleration.x = ((Wire.read() << 8 | Wire.read()) / 16384.0);
    acceleration.y = ((Wire.read() << 8 | Wire.read()) / 16384.0);
    acceleration.z = ((Wire.read() << 8 | Wire.read()) / 16384.0);

    float g = 9.82;
    acceleration *= g;

    // #ifdef Serial
    //     Serial.print("Acceleration: ");
    //     Serial.print(acceleration.x);
    //     Serial.print(", ");
    //     Serial.print(acceleration.y);
    //     Serial.print(", ");
    //     Serial.print(acceleration.z);
    // #endif

    // Gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    // value between -250 and 250 degrees per second
    angularVelocity.x = (Wire.read() << 8 | Wire.read()) / 131.0;
    angularVelocity.y = (Wire.read() << 8 | Wire.read()) / 131.0;
    angularVelocity.z = (Wire.read() << 8 | Wire.read()) / 131.0;

    // #ifdef Serial
    //     Serial.print("      Angular velocity: ");
    //     Serial.print(angularVelocity.x);
    //     Serial.print(", ");
    //     Serial.print(angularVelocity.y);
    //     Serial.print(", ");
    //     Serial.println(angularVelocity.z);
    // #endif
}

void Orientation::readFromIMU() {
    readFromIMU(acceleration, angularVelocity);
    
    acceleration -= accelerationOffset;
    angularVelocity -= angularVelocityOffset;

    // #ifdef Serial
    //     Serial.print("Acceleration: ");
    //     Serial.print(acceleration.x);
    //     Serial.print(", ");
    //     Serial.print(acceleration.y);
    //     Serial.print(", ");
    //     Serial.print(acceleration.z);
    //     Serial.print("      Angular velocity: ");
    //     Serial.print(angularVelocity.x);
    //     Serial.print(", ");
    //     Serial.print(angularVelocity.y);
    //     Serial.print(", ");
    //     Serial.println(angularVelocity.z);
    // #endif
}

void Orientation::update(float deltaTime) {
    readFromIMU();

    calculateAngles(deltaTime);    
    calculateVelocity(deltaTime);
}

vector3<float> Orientation::calculateAccelerationAngles(const vector3<float>& acceleration) {
    return {atan2(acceleration.y, sqrt(pow(acceleration.x, 2) + pow(acceleration.z, 2))) * 180 / PI,
            atan2(-1 * acceleration.x, sqrt(pow(acceleration.y, 2) + pow(acceleration.z, 2))) * 180 / PI,
            atan2(acceleration.y, acceleration.x) * 180 / PI};
}

void Orientation::calculateAngles(float deltaTime) {
    vector3<float> accelerationAngles = (calculateAccelerationAngles(acceleration) - accelerationAngleOffset);

    float rateOfChange = 0.1;
    angles += ((angularVelocity * deltaTime) + angleError) * rateOfChange;

    accelerationError += ((accelerationAngles - angles) * deltaTime);
    angleError =  (accelerationAngles + accelerationError - angles);
}

void Orientation::calculateVelocity(float deltaTime) {
    float sinPitch = sin(angles.x * PI / 180.0);
    float sinRoll = sin(angles.y * PI / 180.0);
    float sinYaw = sin(angles.z * PI / 180.0);
    float cosPitch = cos(angles.x * PI / 180.0);
    float cosRoll = cos(angles.y * PI / 180.0);
    float cosYaw = cos(angles.z * PI / 180.0);
    
    /*
        Acording to ChatGPT, this rotational matrix transforms the acceleration values to the world frame

        |cos(yaw)cos(pitch)     cos(yaw)sin(pitch)sin(roll)−sin(yaw)cos(roll)    cos(yaw)sin(pitch)cos(roll)+sin(yaw)sin(roll)|
​        |sin(yaw)cos(pitch)     sin(yaw)sin(pitch)sin(roll)+cos(yaw)cos(roll)   sin(yaw)sin(pitch)cos(roll)−cos(yaw)sin(roll) |
​        |−sin(pitch)                       cos(pitch)sin(roll)                             cos(pitch)cos(roll)                |
​
        Looks reasonable, but haven't confirmed
    */

    adjustedAcceleration = {
        // x'
        acceleration.x * (cosYaw * cosPitch) +
        acceleration.y * ((cosYaw * sinPitch * sinRoll) - (sinYaw * cosRoll)) +
        acceleration.z * ((cosYaw * sinPitch * cosRoll) + (sinYaw * sinRoll)),
        
        // y'
        acceleration.x * (sinYaw * cosPitch) +
        acceleration.y * ((sinYaw * sinPitch * sinRoll) + (cosYaw * cosRoll)) +
        acceleration.z * ((sinYaw * sinPitch * cosRoll) - (cosYaw * sinRoll)),

        // z'
        acceleration.x * (-sinPitch) +
        acceleration.y * (cosPitch * sinRoll) +
        acceleration.z * (cosPitch * cosRoll)
    };

    velocity += (adjustedAcceleration * deltaTime);
}

void Orientation::calculateOffsets(uint16_t cycles) {
    angularVelocityOffset = {0, 0, 0};
    accelerationOffset = {0, 0, 0};
    accelerationAngleOffset = {0, 0, 0};
    
    vector3<float> acceleration;
    vector3<float> angularVelocity;

    for (int i = 0; i < cycles; i++) {
        readFromIMU(acceleration, angularVelocity);

        angularVelocityOffset += angularVelocity;
        accelerationOffset += acceleration;

        delay(5);
    }
    
    angularVelocityOffset /= cycles;
    accelerationOffset /= cycles;

    for (int i = 0; i < cycles; i++) {
        readFromIMU(acceleration, angularVelocity);
        acceleration -= accelerationOffset;

        accelerationAngleOffset += calculateAccelerationAngles((acceleration + accelerationOffset));

        delay(5);
    }

    accelerationAngleOffset /= cycles;
}