#ifndef RADIODATA_H_
#define RADIODATA_H_

#define PIPE_ADDRESSES {"00001", "00002"}

#define BUTTON_SOUTH    0b00000001
#define BUTTON_WEST     0b00000010
#define BUTTON_NORTH    0b00000100
#define BUTTON_EAST     0b00001000


struct controllerInstructions {
    uint8_t msg_type; // om det i framtiden implementeras att ge direkta positions instructionen för att flytta på drönaren hade det varit i en annan struct

    uint8_t
        stick_LX, stick_LY,
        stick_RX, stick_RY;

    uint8_t trigger_L, trigger_R;

    uint8_t button_input;
};

bool is_pressed(uint8_t &input, uint8_t &button) {
    return (bool)(input & button);
}

struct droneInfo {
    uint8_t state;

    uint8_t 
        power_NE,
        power_SE,
        power_SW,
        power_NW;

    uint8_t
        gyro_X, gyro_Y, gyro_Z,
        accel_X, accel_Y, accel_Z;
}

#endif