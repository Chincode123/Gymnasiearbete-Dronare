#include "RadioData.h"

bool isPressed(uint8_t &input, uint8_t &button) {
    return (bool)(input & button);
}