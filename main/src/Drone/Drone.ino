#include "Drone.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "../utils/RadioData.h"
#include "MotorControl.h"

RF24 radio(/*SE pin*/, /*CSN pin*/)

gyroscopeData gyroscope;

float motorPower1, motorPower2, motorPower3, motorPower4;
MotorController motorController(motorPower1, motorPower2, motorPower3, motorPower4);

float targetVelocity;
float velocityP = 0; 
float velocityI = 0; 
float velocityD = 0;

float targetPitch;
float pitchP = 0; 
float pitchI = 0; 
float pitchD = 0;

float targetRoll;
float rollP = 0; 
float rollI = 0; 
float rollD = 0;

unsigned long previousTime
double deltaTime;
void setDeltaTime() {
    unsigned long currentTime = micros();
    deltaTime = (double)(currentTime - previousTime) / 1000000;
    previusTime = currentTime;
}

void setup() {
    // Set up radio
    radio.begin();
    radio.openWritingPipe(PIPE_ADDRESSES[1]);
    radio.openReadingPipe(1, PIPE_ADDRESSES[0]);

    // Set up motor controller
    motorController.setTargetValues(targetVelocity, targetPitch, targetRoll);
    motorController.setVelocityConstants(velocityP, velocityI, velocityD);
    motorController.setPitchConstants(pitchP, pitchI, pitchD);
    motorController.setRollConstants(rollP, rollI, rollD);


    previousTime = micros();
}

void loop() {
    setDeltaTime();
}