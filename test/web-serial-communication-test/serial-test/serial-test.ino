#include "RadioData.h"

class Message {
public:
    bool initiated = false;
    uint8_t type, length;

    void set(uint8_t type) {
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

    void reset() {
        initiated = false;
    }
};

class InstructionHandler
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

    // returns: message type
    uint8_t getData(uint8_t *out)
    {
        memcpy(out, readBuffer, message.length);
        return message.type;
    }

    void write(uint8_t* data, uint8_t type) {
      Message outputMessage;
      outputMessage.set(type);

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
    
    void reset() {
      Serial.println("Reset");
      acquiredData = false;
      readIndex = 0;
      message.reset();
    }

    void debug() {
      Serial.println("Start Debug");

      Serial.print("R: ");
      Serial.println(reading);

      Serial.print("AD: ");
      Serial.println(acquiredData);

      Serial.print("I: ");
      Serial.println(readIndex);

      Serial.println("Buffer");
      for (int i = 0; i < 32; i++) {
        Serial.print(readBuffer[i]);
        Serial.print(" ");
      }
      Serial.println("\nEnd Debug");
    }
};

InstructionHandler reader;

void setup()
{
    while (!Serial);
    Serial.begin(9600);
}

uint8_t dataBuffer[32];

void loop()
{
    if (reader.read())
    {
      reader.debug();
        switch (reader.getData(dataBuffer))
        {
        case _MSG_CONTROLLER_INPUT:
            controllerInstructions controller;
            memcpy(&controller, dataBuffer, sizeof(controller));

            reader.write((uint8_t*)&controller, _MSG_CONTROLLER_INPUT);

            Serial.println(controller.stick_X);
            Serial.println(controller.stick_Y);
            Serial.println(controller.power);
            Serial.println(controller.button_input);
            break;
        case _MSG_SET_PID_V:
            PID_Instructions pid_v;
            memcpy(&pid_v, dataBuffer, sizeof(pid_v));

            Serial.println("Velocity");
            Serial.print("P: ");
            Serial.println(pid_v.k_p);
            Serial.print("I: ");
            Serial.println(pid_v.k_i);
            Serial.print("D: ");
            Serial.println(pid_v.k_d);

            break;
        case _MSG_SET_PID_P:
            PID_Instructions pid_p;
            memcpy(&pid_p, dataBuffer, sizeof(pid_p));

            Serial.println("Pitch");
            Serial.print("P: ");
            Serial.println(pid_p.k_p);
            Serial.print("I: ");
            Serial.println(pid_p.k_i);
            Serial.print("D: ");
            Serial.println(pid_p.k_d);
            
            break;
        case _MSG_SET_PID_R:
            PID_Instructions pid_r;
            memcpy(&pid_r, dataBuffer, sizeof(pid_r));
            
            Serial.println("Roll");
            Serial.print("P: ");
            Serial.println(pid_r.k_p);
            Serial.print("I: ");
            Serial.println(pid_r.k_i);
            Serial.print("D: ");
            Serial.println(pid_r.k_d);

            break;
        }
        delay(3000);


        reader.debug();
        reader.reset();
        reader.debug();
        delay(1000);
    }
    else {
      reader.debug();
    }

    delay(1000);
}