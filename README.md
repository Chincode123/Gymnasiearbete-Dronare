# Remote-Controlled Drone

This repository contains the software-part of a multi-person project with the goal of creating a remote-controlled drone using arduino microcontrollers, basic components, and 3D-printers

## Table of Contents

- [Remote-Controlled Drone](#remote-controlled-drone)
  - [Table of Contents](#table-of-contents)
  - [Abstract](#abstract)
  - [Drone](#drone)
    - [How `Drone.ino` Works](#how-droneino-works)
      - [Setup](#setup)
      - [Main-Loop](#main-loop)
        - [Radio-Input](#radio-input)
        - [Fight-Control](#fight-control)
        - [Radio-Output](#radio-output)
      - [Important Helper-Functions](#important-helper-functions)
        - [`setDeltaTime`](#setdeltatime)
        - [`sendRadio`](#sendradio)
        - [Activation and De-activation Functions](#activation-and-de-activation-functions)

## Abstract

....

## Drone

[`Drone.ino`](Drone/Drone.ino) is intended to be used with an `Arduino MKR Zero` alongside: an `MPU 6050` gyroscope/accelerometer, an `nRF24L01` radio device, and a set of `DarwinFPV 1104` brushless motors.

>**_NOTE:_**  Will likely work with other components, with some minor modifications to the code

### How [`Drone.ino`](Drone/Drone.ino) Works

[`Drone.ino`](Drone/Drone.ino) is the sketch-file for the drone and, by using both public `Arduino` libraries and the custom-made [`DroneLibrary`](DroneLibrary), it handles the control flow for the drone.

#### Setup

As with most `Arduino`-sketches, the [`Drone.ino`](Drone/Drone.ino)-file includes a setup-function.

```cpp
void setup() {
    ...

    while(!radio.begin()) { }
    while(!configureRadio(radio)) { }
    radio.openWritingPipe(DRONE_ADDRESS);
    radio.openReadingPipe(1, RECEIVER_ADDRESS);
    radio.startListening();

    ...
}
```
Firstly, the radio tranceiver is configured:
- `radio.begin()`, from the [`RF24`](https://github.com/nRF24/RF24) class, is called in order to initialize the radio transceiver
- `configureRadio(radio)` is called in order to configure the tranceiver's settings
- Writing and reading pipes are opened with adresses defined in [`RadioData.h`](DroneLibrary/RadioData.h)
- `radio.startListening()` is called for the tranceiver to start listening for instructions

> **_NOTE:_** The tranceiver's begin and configuration functions are placed into while loops in order to stop the porgramme from proceeding if any part fails.

```cpp
void setup() {
    ...

    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
    motorController.setVelocityConstants(&PID_Velocity);
    motorController.setPitchConstants(&PID_Pitch);
    motorController.setRollConstants(&PID_Roll);

    pinMode(MOTOR_TL_Pin, OUTPUT);
    pinMode(MOTOR_TR_Pin, OUTPUT);
    pinMode(MOTOR_BR_Pin, OUTPUT);
    pinMode(MOTOR_BL_Pin, OUTPUT);

    ...
}
```
Secondly, the motors are configured
- First, values for the [`MotorController`](DroneLibrary/MotorController.h) class are set
- Then, the pin-mode's for the motor-pins are set to ouput

```cpp
void setup() {
    ...

    previousTime = micros();

    radioLogPush("Connected");

    ...
}

```
Lastley, the time-variable is initialized and a message is pushed that will send immediately when the drone is connected to the controller 

#### Main-Loop

The main controll-flow is found within the `loop`-function and can be devided up into three sections: radio-input, flight-control, and radio-output, but before that, at the top of the block, the `setDeltaTime` function is used to set the current time-delta.

##### Radio-Input

```cpp
void loop() {
  ...

  while (radio.available() && millis() % 20 != 0){
    radio.read(&messageIn, sizeof(messageIn));
    switch (messageIn.messageType) {
      case _MSG_CONTROLLER_INPUT:
        // code...
      case _MSG_ACTIVATE:
        // code...
      case _MSG_SET_PID_P:
        // code...
    }
  }

  ...
}
```

This part of the code periodically checks if a radio message is available and, if so, it reads it and interperates the message. The messages are passed through a switch that checks their message-type (as defined in [`RadioData.h`](DroneLibrary/RadioData.h)) and executes a set of instructions based on what type of message it is.

##### Fight-Control

```cpp
void loop() {
  ...

  if (activated) {
      orientation.update(deltaTime);
      motorController.calculatePower(orientation.velocity.z, orientation.angles.x, orientation.angles.y, deltaTime);

      analogWrite(MOTOR_TL_Pin, uint8_t(127 + motorPowerTL));   
      analogWrite(MOTOR_TR_Pin, uint8_t(127 + motorPowerTR));   
      analogWrite(MOTOR_BR_Pin, uint8_t(127 + motorPowerBR));   
      analogWrite(MOTOR_BL_Pin, uint8_t(127 + motorPowerBL));
    }
    else {
      if (millis() % 1000 == 0) {
        radioLogPush("Waiting for activation");
      }
    }

  ...
}
```
This part of the code updates the drone's sensor readings and calculates the required power for each motor, if the drone is activated. 
- Firstly, the `update`-method from the [Orientation](DroneLibrary/Orientation.h)-class is called in order to collect, and proccess, data from the drone's gyroscope.
- Secondly, the new values are passed into the [`MotorController`](DroneLibrary/MotorController.h)-class' `calculatePower`-method in order to calculate the optimal motor-powers for the current moment.
- Lastley, the `Arduino` `analogWrite`-function is used with digital pins in order to create a `PWM`-signal to the motors.

If the drone isn't activated, it periodically sends a message to the controller which displays that it is connected and ready to be activated.

##### Radio-Output

```cpp
void loop() {
  ...

  if (millis() % 20 == 0) sendRadio();

  sequenceTelemetry();

...
}
```

This part of the code controlls the radio-output of the drone. It periodically sends its saved up output-messages, saved in a [`RadioSendStack`](DroneLibrary/RadioSendStack.h)-object, and sequences new telemetry-messages with the `sequenceTelemetry`-function.

```cpp
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
```

#### Important Helper-Functions

##### `setDeltaTime`

```cpp
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
```
It is used in order to set the current time-delta using the `Arduino's` `micros`-function.

##### `sendRadio`

```cpp
void sendRadio() {
    radio.stopListening();
    while (sendStack.getCount() > 0) {
        RadioMessage message = sendStack.pop();
        bool result = radio.write(&message, sizeof(message));

        if (!result){
            sendStack.push(message);
            break;
        }
    }
    radio.startListening();
}
```
It begins by stoping the radio-device from listening to messages, which is required in order to write messages, and enters into a loop that continues until either:
- There are no more messages to send.
- A message fails to send.

It ends by reactivating the listening capabilities of the radio-device.

##### Activation and De-activation Functions

```cpp
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
```

The activation and de-activation functions start of by calling the begin and end methods on the [`Orientation`](DroneLibrary/Orientation.h)-class respectively, and sets an activation-flag to be either true or false respectively. After, they proceed to either ramp up or down the motors to or from 50% respectively. They end of by queueing a message to be sent by radio, informing the user that the drone is either activated or deactivated. Additionally, the deactivation function also ensures that all the motor-pins are turned off completely.