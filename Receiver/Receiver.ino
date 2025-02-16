#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <InstructionHandler.h>

// #define DEBUG

#define CE_PIN 7
#define CSN_PIN 8

RF24 radio(CE_PIN, CSN_PIN, 4000000);

constexpr baudRate = 115200;
InstructionHandler instructionHandler;
uint8_t readBuffer[32];

RadioMessage messageOut, messageIn;

long previousTime;

void setup()
{
    Serial.begin(baudRate);
    while (!Serial);

    if (!radio.begin()){
        Serial.println(F("Radio hardware not responding!"));
        while (true);
    }
    if(!configureRadio(radio)) {
        Serial.println("Radio configuration failed");
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

    // Serial input
    if (instructionHandler.read())
    {
        uint8_t messageType = instructionHandler.getData(readBuffer);
        messageOut.messageType = messageType;
        memcpy(messageOut.dataBuffer, &readBuffer, sizeof(messageOut.dataBuffer));
        bool result = send();

        switch (messageType) {
        case _MSG_CONTROLLER_INPUT:
            if (result) {
                controllerInstructions controller;
                memcpy(&controller, messageOut.dataBuffer, sizeof(controller));
                Serial.print("x:");
                Serial.print((float)controller.stick_X / 127);
                Serial.print(" y:");
                Serial.print((float)controller.stick_Y / 127);
                Serial.print(" power:");
                Serial.println((float)controller.power / 127);
              }
            else {
              // receiverPrint("Faild to send: Controller input");
              }
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
            receiverPrint("Sent: Activation request");
          else 
            receiverPrint("Failed to send: Activation request");
          break;
        case _MSG_DEACTIVATE:
          if (result)
            receiverPrint("Sent: Deactivation request");
          else 
            receiverPrint("Failed to send: Deactivation request");
          break;
        case _MSG_REQUEST_PID_V:
        case _MSG_REQUEST_PID_P:
        case _MSG_REQUEST_PID_R:
        case _MSG_REQUEST_TARGET_RANGES:
          if (result)
            receiverPrint("Sent: Request");
          else
            receiverPrint("Failed to send: Request");
          break;
        // TODO: Add other messages
        // default:
        //     if (result)
        //       receiverPrint("Sent: TYPE=UNKNOWN");
        //     else
        //       receiverPrint("Failed to send: TYPE=UNKNOWN");
        //     break;
        }
    }

    // Radio input
    if (radio.available()) {
        Serial.end();
        radio.read(&messageIn, sizeof(messageIn));
        Serial.begin(baudRate);

        switch (messageIn.messageType) {
          case _MSG_DRONE_LOG:
            dronePrint((const char*)messageIn.dataBuffer);
            break;
          default:
            instructionHandler.write(messageIn.dataBuffer, messageIn.messageType);
        }
    }

    instructionHandler.write((uint8_t*)&deltaTime, _MSG_RECEIVER_DELTATIME);
}

bool send()
{
    Serial.end();
    #ifdef DEBUG
      Serial.println("trying to send");
    #endif

    radio.stopListening();
    bool result = radio.write(&messageOut.dataBuffer, sizeof(messageOut));
    radio.startListening();

    Serial.begin(baudRate);

    #ifdef DEBUG
      if (result)
        Serial.println("successfully sent");
      else
        Serial.println("failed to send");
    #endif
    
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