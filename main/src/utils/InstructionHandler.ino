#include "InstructionHandler.h"
#include "RadioData.h"

bool Message::set(uint8_t& type) {
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

        default:
            return false;
    }

    return true;
}

void Message::reset() {
    initiated = false;
}

bool InstructionReader::read() {
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

uint8_t InstructionReader::getData(uint8_t* out) {
    memcpy(out, readBuffer, message.length);
    uint8_t messageType = message.type;
    acquiredData = false;
    readIndex = 0;
    message.reset();
    
    return messageType;
}

void InstructionWriter::write(uint8_t* data, uint8_t& type) {
    message.set(type);

    uint8_t* output = malloc(message.length + 2);

    *output = 33;
    *(output + 1) = type;

    for (int i = 0; i < message.length; i++) {
        *(output + i + 2) = *(data + i);
    }

    // temp
    Serial.write(output, message.length + 2);

    free(output);
}