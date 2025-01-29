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

long previousTime;

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    if (!radio.begin()){
        Serial.println(F("Radio hardware not responding!"));
        while (true);
    }
    if(!configureRadio(radio)) {
        while (true);
    }
    radio.openWritingPipe(RECEIVER_ADDRESS);
    radio.openReadingPipe(1, DRONE_ADDRESS);
    radio.startListening();

    previousTime = millis();
}

void loop()
{
    float deltaTime = (float)(millis() - previousTime) / 1000;
    previousTime = millis();

    if (instructionHandler.read())
    {
        uint8_t messageType = instructionHandler.getData(readBuffer);
        messageOut.messageType = messageType;
        memcpy(messageOut.dataBuffer, &readBuffer, sizeof(messageOut.dataBuffer));
        bool result = send();

        switch (messageType) {
        case _MSG_DRONE_LOG:
            memcpy(&messageIn, &readBuffer, sizeof(messageIn));
            dronePrint((const char*)messageIn.dataBuffer);
            break;
        case _MSG_CONTROLLER_INPUT:
            if (result)
              receiverPrint("Sent: controller input");
            else
              receiverPrint("Faild to send: controller input");
            break;
        case _MSG_SET_PID_V:
            if (result)
              receiverPrint("Sent: PID-Velocity");
            else
              receiverPrint("Failed to send: PID-Velocity");
          break;
        case _MSG_SET_PID_P:
          if (result)
              receiverPrint("Sent: PID-Pitch");
            else
              receiverPrint("Failed to send: PID-Pitch");
          break;
        case _MSG_SET_PID_R:
            if (result)
              receiverPrint("Sent: PID-Roll");
            else
              receiverPrint("Failed to send: PID-Roll");
          break;
        case _MSG_SET_TARGET_RANGES:
          if (result)
              receiverPrint("Sent: Target ranges");
            else
              receiverPrint("Failed to send: Target ranges");
          break;
        case _MSG_ACTIVATE:
          if (result)
            receiverPrint("Sent: activation request");
          else 
            receiverPrint("Failed to send: activation request");
          break;
        case _MSG_DEACTIVATE:
          if (result)
            receiverPrint("Sent: deactivation request");
          else 
            receiverPrint("Failed to send: deactivation request");
          break;
        case _MSG_REQUEST_PID_V:
        case _MSG_REQUEST_PID_P:
        case _MSG_REQUEST_PID_R:
        case _MSG_REQUEST_TARGET_RANGES:
          if (result)
            receiverPrint("Sent: request");
          else
            receiverPrint("Failed to send: request");
          break;
        // TODO: Add other messages
        default:
            if (result)
              receiverPrint("Sent: TYPE=UNKNOWN");
            else
              receiverPrint("Failed to send: TYPE=UNKNOWN");
            break;
        }
    }

    if (radio.available()) {
        radio.read(&messageIn, sizeof(messageIn));
        instructionHandler.write(messageIn.dataBuffer, messageIn.messageType);
    }

    messageOut.messageType = _MSG_RECEIVER_DELTATIME;
    memcpy(messageOut.dataBuffer, &deltaTime, sizeof(deltaTime));
    send();
}

bool send()
{
    radio.stopListening();
    bool result = radio.write(messageOut.dataBuffer, sizeof(messageOut));
    radio.startListening();
    if (result)
    {
        instructionHandler.acknowledge();
    }
    return result;
}

void receiverPrint(const char* message) {
  Serial.print("[Receiver] ");
  Serial.println(message);
}

void dronePrint(const char* message) {
  Serial.print("[Drone] ");
  Serial.println(message);
}