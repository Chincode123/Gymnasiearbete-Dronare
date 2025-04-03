#include "InstructionHandler.h"
#include "RadioData.h"

Message::Message() {}

Message::Message(uint8_t type) {
    set(type);
}

bool Message::set(uint8_t type) {
    initiated = true;
    this->type = type;
    switch (type) {
        case _MSG_CONTROLLER_INPUT:
            length = _MSG_LENGHT_CONTROLLER;
            break;
        case _MSG_SET_PID_V:
        case _MSG_SET_PID_P:
        case _MSG_SET_PID_R:
            length = _MSG_LENGHT_PID;
            break;
        case _MSG_SET_TARGET_RANGES:
            length = _MSG_LENGTH_TARGET_RANGES;
            break;
        break;
        case _MSG_DRONE_VELOCITY:
            length = _MSG_LENGTH_VELOCITY;
            break;
        case _MSG_DRONE_ACCELERATION:
            length = _MSG_LENGTH_ACCELERATION;
            break;
        case _MSG_DRONE_ANGULAR_VELOCITY:
            length = _MSG_LENGTH_ANGULAR_VELOCITY;
            break;
        case _MSG_DRONE_ANGLES:
            length = _MSG_LENGTH_ANGLES;
            break;
        case _MSG_DRONE_LOG:
            length = _MSG_LENGTH_DRONE_LOG;
            break;
        case _MSG_DRONE_DELTATIME:
        case _MSG_RECEIVER_DELTATIME:
            length = _MSG_LENGTH_DELTATIME;
            break;
        case _MSG_DRONE_MOTOR_POWERS:
            length = _MSG_LENGTH_MOTOR_POWERS;
            break;
        case _MSG_ACKNOWLEDGE:
            length = _MSG_LEGNTH_ACKNOWLEDGE;
        break;
        case _MSG_CONNECTION_STATUS:
            length = _MSG_LENGTH_CONNECTION_STATUS;
            break;
        case _MSG_REQUEST_PID_V:
        case _MSG_REQUEST_PID_P:
        case _MSG_REQUEST_PID_R:
        case _MSG_REQUEST_TARGET_RANGES:
        case _MSG_ACTIVATE:
        case _MSG_DEACTIVATE:
            length = 0;
        default:
            return false;
    }

    return true;
}

void Message::reset() {
    initiated = false;
}

bool InstructionHandler::read() {
    while (Serial.available() > 0 && !acquiredData) {
            uint8_t receivedByte = Serial.read();
            if (reading) {
                if (!message.initiated) {
                    if (!message.set(receivedByte)) {
                        // Message is invalid
                        reading = false;
                    }
                }
                else if (readIndex < message.length - 1) {
                    readBuffer[readIndex++] = receivedByte;
                }
                else {
                    readBuffer[readIndex] = receivedByte;
                    reading = false;
                    acquiredData = true;
                    return true;
                }
            }
            else if (receivedByte == startMarker) {
                reading = true;
            }
    }

    return false;
}

uint8_t InstructionHandler::getData(void* out) {
    memcpy(out, readBuffer, message.length);
    uint8_t messageType = message.type;
    acquiredData = false;
    readIndex = 0;
    memset(readBuffer, 0, sizeof(readBuffer));
    message.reset();
    
    return messageType;
}

void InstructionHandler::write(uint8_t* data, uint8_t type) {
    Message messageOut(type);

    uint8_t* output = (uint8_t*)malloc(messageOut.length + 2);

    *output = 33;
    *(output + 1) = type;

    for (int i = 0; i < messageOut.length; i++) {
        *(output + i + 2) = *(data + i);
    }

    // temp
    Serial.write(output, messageOut.length + 2);

    free(output);
}

void InstructionHandler::acknowledge(uint8_t messageType) {
    write(&messageType, _MSG_ACKNOWLEDGE);
}