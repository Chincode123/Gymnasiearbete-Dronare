#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <InstructionHandler.h>

#define CE_PIN 7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN, 4000000);

InstructionHandler instructionHandler;
uint8_t readBuffer[31];

RadioMessage messageOut, messageIn;

void setup()
{
    Serial.begin(9600);
    while (!Serial);

    if (!radio.begin()){
        Serial.println(F("radio hardware not responding!"));
        while (true);
    }
    if(!configureRadio(radio)) {
        while (true);
    }
    radio.openWritingPipe(RECEIVER_ADDRESS);
    radio.openReadingPipe(1, DRONE_ADDRESS);
    radio.startListening();
}

void loop()
{
    if (instructionHandler.read())
    {
        uint8_t messageType = instructionHandler.getData(readBuffer);
        switch (messageType)
        {
        case _MSG_CONTROLLER_INPUT:
            Serial.println("[RECEIVER] Received controller input");
            break;
        // TODO: Add other messages
        case _MSG_DRONE_LOG:
            memcpy(&messageIn, &readBuffer, sizeof(messageIn));

            Serial.print("[DRONE] ");
            Serial.println((const char*)messageIn.dataBuffer);
        default:
            Serial.println("[RECEIVER] Received");
            break;
        }

        messageOut.messageType = messageType;
        memcpy(messageOut.dataBuffer, &readBuffer, sizeof(messageOut.dataBuffer));
        send();
    }

    if (radio.available()) {
        radio.read(&messageIn, sizeof(messageIn));
        instructionHandler.write(messageIn.dataBuffer, messageIn.messageType);
    }
}

void send()
{
    radio.stopListening();
    bool result = radio.write(messageOut.dataBuffer, sizeof(messageOut));
    radio.startListening();
    if (result)
    {
        Serial.println("[RECEIVER] Radio acknowledgment received");
        instructionHandler.acknowledge();
    }
    else
    {
        Serial.println("[RECEIVER] Radio acknowledgment not received");
    }
}