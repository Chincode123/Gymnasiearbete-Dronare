#ifndef RADIODATA_H_
#define RADIODATA_H_

#include <RF24.h>
#include <Arduino.h>

#define DRONE_ADDRESS       (uint8_t*)"DRONE"  
#define RECEIVER_ADDRESS    (uint8_t*)"RCEVR"

#define _MSG_CONTROLLER_INPUT 0
#define _MSG_SET_PID_V 1
#define _MSG_SET_PID_P 2
#define _MSG_SET_PID_R 3
#define _MSG_REQUEST_PID_V 4
#define _MSG_REQUEST_PID_P 5
#define _MSG_REQUEST_PID_R 6
#define _MSG_SET_TARGET_RANGES 7
#define _MSG_REQUEST_TARGET_RANGES 8
#define _MSG_ACKNOWLEDGE 9
#define _MSG_DRONE_LOG 10
#define _MSG_ACTIVATE 11
#define _MSG_DEACTIVATE 12
#define _MSG_DRONE_VELOCITY 13
#define _MSG_DRONE_ACCELERATION 14
#define _MSG_DRONE_ANGULAR_VELOCITY 15
#define _MSG_DRONE_ANGLES 16
#define _MSG_DRONE_DELTATIME 17
#define _MSG_RECEIVER_DELTATIME 18
#define _MSG_DRONE_MOTOR_POWERS 19

// message length in bytes
#define _MSG_LENGHT_CONTROLLER 3
#define _MSG_LENGHT_PID 12
#define _MSG_LENGTH_TARGET_RANGES 12
#define _MSG_LENGTH_RADIO_MESSAGE 32
#define _MSG_LENGTH_DRONE_LOG 31
#define _MSG_LENGTH_VELOCITY 12
#define _MSG_LENGTH_ACCELERATION 12
#define _MSG_LENGTH_ANGULAR_VELOCITY 12
#define _MSG_LENGTH_ANGLES 12
#define _MSG_LENGTH_DELTATIME 4
#define _MSG_LEGNTH_ACKNOWLEDGE 1
#define _MSG_LENGTH_MOTOR_POWERS 4

struct controllerInstructions {
    int8_t stick_X;
    int8_t stick_Y;
    int8_t power;
};

struct PID_Instructions {
    float k_p, k_i, k_d;
};

struct TargetRangeInstructions {
    float pitchMax, rollMax, verticalVelocityMax;
};

struct RadioMessage {
    uint8_t messageType;
    uint8_t dataBuffer[31];
};

struct MotorPowers {
    int8_t TL, TR, BR, BL;
};

bool configureRadio(RF24& radio);
#endif