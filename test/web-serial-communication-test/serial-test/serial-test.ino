#include "RadioData.h"

class InstructionReader
{
    Message message;
    bool reading = false;
    bool acquiredData = false;
    uint8_t readIndex = 0;
    uint8_t readBuffer[32];
    char startMarker = '<';
    char endMarker = '>';

public:
    bool read()
    {
        while (Serial.available() > 0 && !acquiredData)
        {
            uint8_t receivedByte = Serial.read();
            if (reading)
            { 
                if (!message.instatiated) {
                    message.set(receivedByte);
                }

                if (recivedByte != endMarker){
                    readBuffer[readIndex++] = receivedByte;
                }
                else
                {
                    reading = false;
                    acquiredData = true;
                    return true;
                }
            }
            else if (receivedByte == startMarker) {
                reading = true
            }
        }

        return false;
    }

    // returns: message type
    uint8_t getData(uint8_t *out)
    {
        memcpy(out, readBuffer, message.length);
        acquiredData = false;
        uint8_t messageType = message.type;
        message.reset();
        return messageType;
    }
};

InstructionReader reader;

void setup()
{
    while (!Serial);
    Serial.begin(9600);
}

uint8_t dataBuffer[32];

void loop()
{
    if (reader.read()) {
        switch(reader.getData(dataBuffer))
        {
            case _MSG_CONTROLLER_INPUT:
                controllerInstructions controller = (controllerInstructions)dataBuffer;
                break;
            case _MSG_LENGHT_PID_V:
                PID_Instructions pid_V = (PID_Instructions)dataBuffer;
                break;
            case _MSG_LENGHT_PID_P:
                PID_Instructions pid_p = (PID_Instructions)dataBuffer;
                break;
            case _MSG_LENGHT_PID_R:
                PID_Instructions pid_r = (PID_Instructions)dataBuffer;
                break;
        }
    }
}