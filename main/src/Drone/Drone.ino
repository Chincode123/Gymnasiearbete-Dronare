#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "../utils/RadioData.h"
#include "../utils/MotorControl.h"
#include "../utils/Vectors.h"
#include "../utils/Orientation.h"
#include "../utils/RadioSendStack.h"

#define CE_PIN 6
#define CSN_PIN 7

// Radio
RF24 radio(CE_PIN, CSN_PIN, 4000000);
uint8_t readBuffer[32];
RadioSendStack sendStack;


// Motors
#define MOTOR_TL_Pin 5
#define MOTOR_TR_Pin 4
#define MOTOR_BR_Pin 3
#define MOTOR_BL_Pin 2
uint8_t motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL;
MotorController motorController(motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL);


// PID-values
float targetVelocity;
float velocityP = 0; 
float velocityI = 0; 
float velocityD = 0;
float maxVelocity;

float targetPitch;
float pitchP = 0; 
float pitchI = 0; 
float pitchD = 0;
float maxPitch;

float targetRoll;
float rollP = 0; 
float rollI = 0; 
float rollD = 0;
float maxRoll;


// Delta time
unsigned long previousTime
float deltaTime;
void setDeltaTime();


// Spatial orientation/acceleration/velocity
Orientation orientation;


bool activated = false;


void setup() {
    // Set up radio
    if (!radio.begin()){
        Serial.println(F("radio hardware not responding!"));
        while (true);
    }
    if(!configureRadio(radio)) {
        while (true);
    }
    radio.openWritingPipe(DRONE_ADDRESS);
    radio.openReadingPipe(1, RECEIVER_ADDRESS);

    // Set up motor controller
    motorController.setTargetValues(targetVelocity, targetPitch, targetRoll);
    motorController.setVelocityConstants(velocityP, velocityI, velocityD);
    motorController.setPitchConstants(pitchP, pitchI, pitchD);
    motorController.setRollConstants(rollP, rollI, rollD);

    pinMode(MOTOR_TL_Pin, OUTPUT);
    pinMode(MOTOR_TR_Pin, OUTPUT);
    pinMode(MOTOR_BR_Pin, OUTPUT);
    pinMode(MOTOR_BL_Pin, OUTPUT);
    
    // Initial time
    previousTime = millis();

    consoleLog("Connected", true);
}

void loop() {
    setDeltaTime();
    
    // Radio read
    if (radio.available()) {
        radio.read(&readBuffer, sizeof(readBuffer));

        RadioMessage message;
        memcpy(&message, &readBuffer, sizeof(message));

        switch (message.type) {
            case _MSG_CONTROLLER_INPUT:
                controllerInstructions controller;
                memcpy(&controller, &message.dataBuffer, sizeof(controller));
                
                targetVelocity = maxVelocity * (0.5 + (float)controller.power / 255);
                targetPitch = maxPitch * ((float)controller.stick_X / 127);
                targetRoll = maxRoll * ((float)controller.stick_Y / 127);
                break;
            case _MSG_ACTIVATE:
                if (activated) break;
                activated = true;

                orientation.begin(500);

                // Ramp up motors to 50%
                uint8_t power = 0;
                float time = 0;
                while (time < 1) {
                    setDeltaTime();
                    analogWrite(MOTOR_TL_Pin, power);
                    analogWrite(MOTOR_TR_Pin, power);
                    analogWrite(MOTOR_BR_Pin, power);
                    analogWrite(MOTOR_BL_Pin, power);

                    time += deltaTime;
                    power = 128 * time;
                }
                
                consoleLog("Activation complete", true);
                break;
            case _MSG_DEACTIVATE:
                if (!activated) break;
                activated = false;

                orientation.end();

                // Ramp down motors to 0%
                uint8_t power = 0;
                float time = 0;
                while (time < 1) {
                    setDeltaTime();
                    analogWrite(MOTOR_TL_Pin, power);
                    analogWrite(MOTOR_TR_Pin, power);
                    analogWrite(MOTOR_BR_Pin, power);
                    analogWrite(MOTOR_BL_Pin, power);

                    time += deltaTime;
                    power = 128 * (1 - time);
                }
                digitalWrite(MOTOR_TL_Pin, LOW);
                digitalWrite(MOTOR_TR_Pin, LOW);
                digitalWrite(MOTOR_BR_Pin, LOW);
                digitalWrite(MOTOR_BL_Pin, LOW);

                consoleLog("Deactivated");
                break;
            // TODO: add all cases
            default:
                consoleLog("Error interpreting messageType", false);
                break;
        }
    }

    if (activated) {


        orientation.update(deltaTime);
        motorController.calculatePower(orientation.velocity, orientation.angles.x, orientation.angles.y, deltaTime);
        analogWrite(MOTOR_TL_Pin, motorPowerTL);   
        analogWrite(MOTOR_TR_Pin, motorPowerTR);   
        analogWrite(MOTOR_BR_Pin, motorPowerBR);   
        analogWrite(MOTOR_BL_Pin, motorPowerBL);
    }
    else {
        if (millis() % 5000 == 0) {
            consoleLog("Waiting for activation");
        }
    }

    // Output
    sendRadio();
}

void setDeltaTime() {
    unsigned long currentTime = millis();
    deltaTime = (float)(currentTime - previousTime) / 1000;
    previusTime = currentTime;
}

void sendRadio() {
    while (sendStack.count > 0 && !radio.available()) {
        radio.stopListening();
        radioStackElement* data = sendStack.pop();
        bool result = radio.write(data->value, data->size);
        radio.startListening();

        if (!result){
            sendStack.push(data->value, data->size);
            break;
        }
    }
}

void consoleLog(const char* message, bool important) {
    #define DRONE_LOG
    RadioMessage logMessage;
    logMessage.messageType = _MSG_DRONE_LOG;
    uint8_t messageLength = strlen(message);
    messageLength = (messageLength < 31) ? messageLength : 31;
    memcpy(&logMessage.dataBuffer, &message, messageLength);
    
    if (impotrant)
        sendStack.push(&logMessage, sizeof(logMessage));
    else
        sendStack.queue(&logMessage, sizeof(logMessage));
}