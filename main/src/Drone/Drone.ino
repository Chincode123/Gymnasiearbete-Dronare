#include "Drone.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "../utils/RadioData.h"
#include "MotorControl.h"

#define CE_PIN 6
#define CSN_PIN 7

RF24 radio(CE_PIN, CSN_PIN);


// Motors
#define MOTOR1_Pin 
#define MOTOR2_Pin 
#define MOTOR3_Pin 
#define MOTOR4_Pin 
uint8_t motorPower1, motorPower2, motorPower3, motorPower4;
MotorController motorController(motorPower1, motorPower2, motorPower3, motorPower4);


// PID-values
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

// Delta time
unsigned long previousTime
float deltaTime;
void setDeltaTime();

// Spatial position
vector velocity;
vector angles; // x => pitch, y => roll, z => yaw;
void setGyroscopeValues();

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

    pinMode(MOTOR1_Pin, OUTPUT);
    pinMode(MOTOR2_Pin, OUTPUT);
    pinMode(MOTOR3_Pin, OUTPUT);
    pinMode(MOTOR4_Pin, OUTPUT);
    
    // Initial time
    previousTime = micros();
}

void loop() {
    setDeltaTime();
    
    setGyroscopeValues();
    motorController.calculatePower(velocity.y, angles.x, angles.y, deltaTime);
    analogWrite(MOTOR1_Pin, motorPower1);   
    analogWrite(MOTOR2_Pin, motorPower2);   
    analogWrite(MOTOR3_Pin, motorPower3);   
    analogWrite(MOTOR4_Pin, motorPower4);
}

void setDeltaTime() {
    unsigned long currentTime = micros();
    deltaTime = (float)(currentTime - previousTime) / 1000000;
    previusTime = currentTime;
}