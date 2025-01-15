#include "InstructionHandler.h"
#include "RadioData.h"

void Message::set(uint8_t type) {
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
    }
}

void Message::reset() {
    initiated = false;
}


bool InstructionReader::read() {
    while (Serial.available() > 0 && !acquiredData)
        {
            uint8_t receivedByte = Serial.read();
            if (reading) 
            {
                if (!message.initiated)
                {
                    message.set(receivedByte);
                }
                else if (receivedByte != endMarker)
                {
                    readBuffer[readIndex++] = receivedByte;
                }
                else
                {
                    reading = false;
                    acquiredData = true;
                    return true;
                }
            }
            else if (receivedByte == startMarker)
            {
                reading = true;
            }
        }

        return false;
}

uint8_t InstructionReader::getData(uint8_t* out) {
    memcpy(out, readBuffer, message.length);
    return message.type;
}

void InstructionWriter::write(uint8_t* data, uint8_t type) {
    message.set(type);

    uint8_t* output = malloc(outputMessage.length + 3);

    *output = 60;
    *(output + 1) = type;
    *(output + outputMessage.length + 2) = 62;

    for (int i = 0; i < outputMessage.length; i++) {
        *(output + i + 2) = *(data + i);
    }

    Serial.write(output, outputMessage.length + 3);

    free(output);
}