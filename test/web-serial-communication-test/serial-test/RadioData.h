#ifndef RADIODATA_H_
#define RADIODATA_H_

#define PIPE_ADDRESSES {"00001", "00002"}

#define BUTTON_0    0b00000001
#define BUTTON_1    0b00000010
#define BUTTON_2    0b00000100
#define BUTTON_3    0b00001000
#define BUTTON_4    0b00010000
#define BUTTON_5    0b00100000
#define BUTTON_6    0b01000000
#define BUTTON_7    0b10000000

#define _MSG_CONTROLLER_INPUT 0
#define _MSG_SET_PID_V 1
#define _MSG_SET_PID_P 2
#define _MSG_SET_PID_R 3

// message length in bytes
#define _MSG_LENGHT_CONTROLLER 4
#define _MSG_LENGHT_PID 12

struct controllerInstructions {
    // devide by 127
    int8_t
        stick_X, stick_Y,
        power;

    uint8_t button_input;
};

struct PID_Instructions {
    float k_p, k_i, k_d;
};

struct gyroscopeValues {
    float   pitch,
            roll,
            yaw;

    float   accX,
            accY,
            accZ;
};

struct gyroscopeErrors{
    float   accErrorX, 
            accErrorY, 
            gyroErrorX, 
            gyroErrorY, 
            gyroErrorZ;
};

struct gyroscopeData{
    gyroscopeValues values;
    gyroscopeErrors errors;
};

struct droneInfo {
    uint8_t state;

    uint8_t 
        power_NE,
        power_SE,
        power_SW,
        power_NW;

    gyroscopeData gyroscope;
};

bool isPressed(uint8_t &input, uint8_t &button);

#endif