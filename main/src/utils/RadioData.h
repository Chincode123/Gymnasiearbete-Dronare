#ifndef RADIODATA_H_
#define RADIODATA_H_

#define PIPE_ADDRESSES {"00001", "00002"}

#define BUTTON_SOUTH    0b00000001
#define BUTTON_WEST     0b00000010
#define BUTTON_NORTH    0b00000100
#define BUTTON_EAST     0b00001000

struct controllerInstructions {
    // om det i framtiden implementeras att ge direkta positions instructionen för att flytta på drönaren hade det varit i en annan struct
    uint8_t msg_type; 

    int8_t
        stick_LX, stick_LY,
        triggerValues;

    uint8_t button_input;
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