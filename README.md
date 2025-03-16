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

## Abstract

....

## Drone

[`Drone.ino`](Drone/Drone.ino) is intended to be used with an `Arduino MKR Zero` alongside: an `MPU 6050` gyroscope/accelerometer, an `nRF24L01` radio device, and a set of `DarwinFPV 1104` brushless motors.

>**_NOTE:_**  Will likely work with other components, with some minor modifications to the code

### How [`Drone.ino`](Drone/Drone.ino) Works

[`Drone.ino`](Drone/Drone.ino) is the sketch-file for the drone and, by using both public `Arduino` libraries and the custom-made [`DroneLibrary`](DroneLibrary), it handles the control flow for the drone.

#### Setup

As with most `Arduino` sketches, the `Drone.ino` file includes a setup function.

```cpp
void setup() {
    ...

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

    ...
}
```
Firstly, the radio tranceiver is configured:
- `radio.begin()`, from the [`RF24`](https://github.com/nRF24/RF24) class, is called to initialize the radio transceiver
- `configureRadio(radio)` is called to configure the tranceiver's settings
- Writing and reading pipes are opened with adresses defined in [`RadioData.h`](DroneLibrary/RadioData.h)
- `radio.startListening()` is called for the tranceiver to start listening for instructions

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

    radioLog("Connected", true);

    ...
}

```
Lastley, the time variable is initialized and a message is pushed that will send immediately when the drone is connected to the controller 

#### Main-Loop