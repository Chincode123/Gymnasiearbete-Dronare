#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RadioData.h>
#include <MotorController.h>
#include <Vectors.h>
#include <Orientation.h>
#include <RadioSendStack.h>

#define DEBUG

#define CE_PIN 7
#define CSN_PIN 6

// Radio
RF24 radio(CE_PIN, CSN_PIN, 4000000);
uint8_t readBuffer[32];
RadioSendStack sendStack;
RadioMessage messageIn, messageOut;

// Motors
#define MOTOR_TL_Pin 5
#define MOTOR_TR_Pin 4
#define MOTOR_BR_Pin 3
#define MOTOR_BL_Pin 2
int8_t motorPowerTL = 0, motorPowerTR = 0, motorPowerBR = 0, motorPowerBL = 0;
MotorController motorController(motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL);


// PID-values
float targetVelocity;
float maxVelocity;
PID_Instructions PID_Velocity;

float targetPitch;
float maxPitch;
PID_Instructions PID_Pitch;

float targetRoll;
float maxRoll;
PID_Instructions PID_Roll;


// Delta time
unsigned long previousTime;
float deltaTime;
void setDeltaTime();


// Spatial orientation/acceleration/velocity
const int MPU = 0x68;
Orientation orientation(MPU);


bool activated = false;

void setup() {
    #ifdef DEBUG
      Serial.begin(115200);
      while (!Serial);
    #endif

    // Set up radio
    while(!radio.begin()) {
      #ifdef DEBUG
        Serial.println("radio hardrware issue");
      #endif
    }
    while(!configureRadio(radio)) {
      #ifdef DEBUG
        Serial.println("radio config issue");
      #endif
    }
    radio.openWritingPipe(DRONE_ADDRESS);
    radio.openReadingPipe(1, RECEIVER_ADDRESS);
    radio.startListening();

    // Set up motor controller
    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
    motorController.setVelocityConstants(PID_Velocity);
    motorController.setPitchConstants(PID_Pitch);
    motorController.setRollConstants(PID_Roll);

    pinMode(MOTOR_TL_Pin, OUTPUT);
    pinMode(MOTOR_TR_Pin, OUTPUT);
    pinMode(MOTOR_BR_Pin, OUTPUT);
    pinMode(MOTOR_BL_Pin, OUTPUT);
    
    // Initial time
    previousTime = micros();

    radioLogPush("Connected");

    #ifdef DEBUG
      Serial.println("completed setup");
    #endif
}

void loop() {
    setDeltaTime();
    
    // Radio read
    if (radio.available() && millis() % 20 != 0) {
        #ifdef DEBUG
          Serial.println("radio available");
        #endif
        
        radio.read(&readBuffer, sizeof(readBuffer));
        memcpy(&messageIn, &readBuffer, sizeof(messageIn));

        #ifdef DEBUG
          printRadioMessage(messageIn);
        #endif

        switch (messageIn.messageType) {
            case _MSG_CONTROLLER_INPUT:
                controllerInstructions controller;
                memcpy(&controller, messageIn.dataBuffer, sizeof(controller));
                
                #ifdef DEBUG
                  Serial.print("x:");
                  Serial.print((float)controller.stick_X / 127);
                  Serial.print(" y:");
                  Serial.print((float)controller.stick_Y / 127);
                  Serial.print(" power:");
                  Serial.println((float)controller.power / 127);
                #endif

                targetVelocity = maxVelocity * (float)controller.power / 127;
                targetPitch = maxPitch * ((float)controller.stick_X / 127);
                targetRoll = maxRoll * ((float)controller.stick_Y / 127);
                break;
            case _MSG_ACTIVATE:
                activate();
                break;
            case _MSG_DEACTIVATE:
                deactivate();
                break;
            case _MSG_SET_PID_V:
                memcpy(&PID_Velocity, messageIn.dataBuffer, sizeof(PID_Velocity));

                #ifdef DEBUG
                  Serial.print("PID-V: p:");
                  Serial.print(PID_Velocity.k_p);
                  Serial.print(" i:");
                  Serial.print(PID_Velocity.k_i);
                  Serial.print(" d:");
                  Serial.println(PID_Velocity.k_d);
                #endif

                break;
            case _MSG_SET_PID_P:
                memcpy(&PID_Pitch, messageIn.dataBuffer, sizeof(PID_Pitch));               
                break;
            case _MSG_SET_PID_R:
                memcpy(&PID_Roll, messageIn.dataBuffer, sizeof(PID_Roll));
                break;
            case _MSG_REQUEST_PID_V:
                memcpy(messageOut.dataBuffer, &PID_Velocity, sizeof(PID_Velocity));
                messageOut.messageType = _MSG_SET_PID_V;
                sendStack.push(messageOut);

                #ifdef DEBUG
                  Serial.println("Requesting pid-v");
                #endif

                break;
            case _MSG_REQUEST_PID_P:
                memcpy(messageOut.dataBuffer, &PID_Pitch, sizeof(PID_Pitch));
                messageOut.messageType = _MSG_SET_PID_P;
                sendStack.push(messageOut);
                break;
            case _MSG_REQUEST_PID_R:
                memcpy(messageOut.dataBuffer, &PID_Roll, sizeof(PID_Roll));
                messageOut.messageType = _MSG_SET_PID_R;
                sendStack.push(messageOut);
                break;
            case _MSG_SET_TARGET_RANGES:
                TargetRangeInstructions targetRangesIn;
                memcpy(&targetRangesIn, messageIn.dataBuffer, sizeof(targetRangesIn));
                maxVelocity = targetRangesIn.verticalVelocityMax;
                maxPitch = targetRangesIn.pitchMax;
                maxRoll = targetRangesIn.rollMax;
                break;
            case _MSG_REQUEST_TARGET_RANGES:
                TargetRangeInstructions targetRangesOut;
                targetRangesOut = {maxPitch, maxRoll, maxVelocity};
                memcpy(messageOut.dataBuffer, &targetRangesOut, sizeof(targetRangesOut));
                messageOut.messageType = _MSG_SET_TARGET_RANGES;
                sendStack.push(messageOut);
                break;
            default:
                radioLogQueue("Error interpreting messageType");
                break;
        }
    }
    #ifdef DEBUG
    else {
      Serial.println("radio not available");
    }
    #endif

    if (activated) {
      #ifdef DEBUG
        Serial.println("activated");
      #endif

        orientation.update(deltaTime);
        motorController.calculatePower(orientation.velocity.z, orientation.angles.x, orientation.angles.y, deltaTime);
        #ifndef DEBUG
          analogWrite(MOTOR_TL_Pin, uint8_t(127 + motorPowerTL));   
          analogWrite(MOTOR_TR_Pin, uint8_t(127 + motorPowerTR));   
          analogWrite(MOTOR_BR_Pin, uint8_t(127 + motorPowerBR));   
          analogWrite(MOTOR_BL_Pin, uint8_t(127 + motorPowerBL));
        #endif
    }
    else {
        #ifdef DEBUG
          Serial.println("not activated");
        #endif

        if (millis() % 1000 == 0) {
            radioLogPush("Waiting for activation");
        }
    }

    // Output
    if (millis() % 20 == 0) sendRadio();

    sequenceTelemetry();
}

void setDeltaTime() {
    unsigned long currentTime = micros();
    if (currentTime < previousTime) {
        deltaTime = (float)(currentTime + (0xFFFFFFFF - previousTime)) / 1000000;
    } else {
        deltaTime = (float)(currentTime - previousTime) / 1000000;
    }
    previousTime = currentTime;
    if (deltaTime == 0) {
        deltaTime = 0.000001;
    }
}

void sendRadio() {
    #ifdef DEBUG
      Serial.print("(before) SendStack count: ");
      Serial.println(sendStack.getCount());
    #endif

    radio.stopListening();
    while (sendStack.getCount() > 0) {
        #ifdef DEBUG 
          Serial.println("attempting to send");
        #endif

        RadioMessage message = sendStack.pop();
        bool result = radio.write(&message, sizeof(message));

        #ifdef DEBUG
          printRadioMessage(message);
        #endif

        if (!result){
            #ifdef DEBUG
              Serial.println("failed to send");
            #endif

            sendStack.push(message);
            break;
        }

        #ifdef DEBUG
          Serial.println("successfully sent");
        #endif
    }
    radio.startListening();
    #ifdef DEBUG
      Serial.print("(after) SendStack count: ");
      Serial.println(sendStack.getCount());
    #endif
}

void radioLogPush(const char* message) {
    RadioMessage logMessage;
    logMessage.messageType = _MSG_DRONE_LOG;
    uint8_t messageLength = strlen(message);
    messageLength = (messageLength < 31) ? messageLength : 31;
    memcpy(logMessage.dataBuffer, message, messageLength);
    sendStack.push(logMessage);
}

void radioLogQueue(const char* message) {
  RadioMessage logMessage;
  logMessage.messageType = _MSG_DRONE_LOG;
  uint8_t messageLength = strlen(message);
  messageLength = (messageLength < 31) ? messageLength : 31;
  memcpy(logMessage.dataBuffer, message, messageLength);
  sendStack.push(logMessage);
}

#ifdef DEBUG
void printRadioMessage(RadioMessage message) {
    Serial.print("Message type: ");
    Serial.println(message.messageType);
    Serial.print("Data: ");
    for (int i = 0; i < sizeof(message.dataBuffer); i++) {
        Serial.print((int)message.dataBuffer[i]);
        Serial.print(" ");
    }
    Serial.print("Length: ");
    Serial.println((int)sizeof(message));
}
#endif

void activate() {
  if (activated) return;
  activated = true;
  orientation.begin(500);
  #ifndef DEBUG
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
  #endif

  radioLogPush("Activation complete");
}

void deactivate() {
  if (!activated) return;
  activated = false;
  orientation.end();
  #ifndef DEBUG
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
  #endif
  digitalWrite(MOTOR_TL_Pin, LOW);
  digitalWrite(MOTOR_TR_Pin, LOW);
  digitalWrite(MOTOR_BR_Pin, LOW);
  digitalWrite(MOTOR_BL_Pin, LOW);
  radioLogPush("Deactivated");
}

template<typename T>
void sequenceVector(vector3<T> vector, uint8_t messageType) {
  messageOut.messageType = messageType;
  float x = float(vector.x);
  float y = float(vector.y);
  float z = float(vector.z);
  memcpy(messageOut.dataBuffer + sizeof(float) * 0, &x, sizeof(float));
  memcpy(messageOut.dataBuffer + sizeof(float) * 1, &y, sizeof(float));
  memcpy(messageOut.dataBuffer + sizeof(float) * 2, &z, sizeof(float));
  sendStack.push(messageOut);
}

void sequenceTelemetry() {
  if (sendStack.getCount() > 0)
    return;

  sequenceVector(orientation.adjustedAcceleration, _MSG_DRONE_ACCELERATION);
  sequenceVector(orientation.velocity, _MSG_DRONE_VELOCITY);
  sequenceVector(orientation.angularVelocity, _MSG_DRONE_ANGULAR_VELOCITY);
  sequenceVector(orientation.angles, _MSG_DRONE_ANGLES);

  messageOut.messageType = _MSG_DRONE_DELTATIME;
  memcpy(messageOut.dataBuffer, &deltaTime, sizeof(deltaTime));
  sendStack.push(messageOut);
}