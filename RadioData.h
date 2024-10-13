#ifndef RADIODATA_H_
#define RADIODATA_H_

#define BUTTON_SOUTH    0b00000001
#define BUTTON_WEST     0b00000010
#define BUTTON_NORTH    0b00000100
#define BUTTON_EAST     0b00001000


struct droneInstructions {
    uint8_t msg_type;

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
}

#endif