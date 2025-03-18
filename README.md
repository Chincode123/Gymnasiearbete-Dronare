# Remote Controlled Drone

This repository contains the software-part of a multi-person project with the goal of creating a remote controlled drone using arduino microcontrollers, basic components, and 3D-printers

## Table of Contents

- [Remote Controlled Drone](#remote-controlled-drone)
  - [Table of Contents](#table-of-contents)
  - [Abstract](#abstract)
  - [Drone](#drone)
    - [Basic Structure](#basic-structure)
      - [Setup](#setup)
      - [Main-Loop](#main-loop)
        - [Radio-Input](#radio-input)
        - [Fight-Control](#fight-control)
        - [Radio-Output](#radio-output)
      - [Important Helper Functions](#important-helper-functions)
        - [`setDeltaTime`](#setdeltatime)
        - [`sendRadio`](#sendradio)
        - [Activation and De-activation Functions](#activation-and-de-activation-functions)
    - [Important Classes](#important-classes)
      - [`PID`](#pid)
      - [`MotorController`](#motorcontroller)
      - [`Orientation`](#orientation)
      - [`RadioSendStack`](#radiosendstack)
      - [Custom Data Types](#custom-data-types)
        - [`vector3<T>`](#vector3t)
        - [`SmoothValue`](#smoothvalue)
        - [Other](#other)
    - [Summery of Drone](#summery-of-drone)
  - [Receiver](#receiver)
    - [Basic Structure](#basic-structure-1)
    - [Important Classes](#important-classes-1)
      - [`InstructionHandler`](#instructionhandler)
    - [Summery of Receiver](#summery-of-receiver)
  - [Controller](#controller)
    - [Basic Structure](#basic-structure-2)
    - [Important Classes](#important-classes-2)
    - [Summery of Controller](#summery-of-controller)
  - [Simulation](#simulation)
  - [Summery](#summery)

## Abstract

....

## Drone

[`Drone.ino`](Drone/Drone.ino) is intended to be used with an `Arduino MKR Zero` alongside: an `MPU 6050` gyroscope/accelerometer, an `nRF24L01` radio device, and a set of `DarwinFPV 1104` brushless motors.

>**_NOTE:_**  Will likely work with other components, with some minor modifications to the code

### Basic Structure

[`Drone.ino`](Drone/Drone.ino) is the sketch file for the drone and, by using both public `Arduino` libraries and the custom-made [`DroneLibrary`](DroneLibrary), it handles the control flow for the drone.

#### Setup

As with most `Arduino` sketches, the [`Drone.ino`](Drone/Drone.ino) file includes a setup-function.

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
- First, values for the [`MotorController`](#motorcontroller) class are set
- Then, the pin-mode's for the motor-pins are set to ouput

```cpp
void setup() {
    ...

    previousTime = micros();

    radioLogPush("Connected");

    ...
}

```
Lastley, the time variable is initialized and a message is pushed that will send immediately when the drone is connected to the controller 

#### Main-Loop

The main controll-flow is found within the `loop` function and can be devided up into three sections: radio input, flight control, and radio output, but before that, at the top of the block, the `setDeltaTime` function is used to set the current time-delta.

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

This part of the code periodically checks if a radio message is available and, if so, it reads it and interperates the message. The messages are passed through a switch that checks their message type (as defined in [`RadioData.h`](DroneLibrary/RadioData.h)) and executes a set of instructions based on what type of message it is.

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
- Firstly, the `update` method from the [Orientation](DroneLibrary/Orientation.h) class is called in order to collect, and proccess, data from the drone's gyroscope.
- Secondly, the new values are passed into the [`MotorController`](#motorcontroller) class' `calculatePower` method in order to calculate the optimal motor powers for the current moment.
- Lastley, the `Arduino` `analogWrite` function is used with digital pins in order to create a `PWM` signal to the motors.

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

This part of the code controlls the radio-output of the drone. It periodically sends its saved up output-messages, saved in a [`RadioSendStack`](#radiosendstack) object, and sequences new telemetry messages with the `sequenceTelemetry` function.

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

#### Important Helper Functions

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
It begins by stoping the radio device from listening to messages, which is required in order to write messages, and enters into a loop that continues until either:
- There are no more messages to send.
- A message fails to send.

It ends by reactivating the listening capabilities of the radio device.

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

The activation and de-activation functions start of by calling the begin and end methods on the [`Orientation`](DroneLibrary/Orientation.h) class respectively, and sets an activation flag to be either true or false respectively. After, they proceed to either ramp up or down the motors to or from 50% respectively. They end of by queueing a message to be sent by radio, informing the user that the drone is either activated or deactivated. Additionally, the deactivation function also ensures that all the motor pins are turned off completely.

### Important Classes

For the drone to function propperly, multiple classes were created to handle critical tasks.

#### `PID`

The [`PID`](DroneLibrary/PIDController.h) class implements a standard PID-controller with a scalar value for each of the current **proportional** error, the cumulative **integral** of the error, and the current **dirivitive** of the error

**To use the class:**
1. First, create an object. 
   ```cpp
    PID pid();
   ```
2. Then, use the `setConstants` method to set a pointer to the scalar values, and use the `setTarget` method to set a pointer to the PID-controller's target value. 
   ```cpp
    float p = ...
    float i = ...
    float d = ...
    pid.setConstants(&p, &i, &d);

    float target = ...

    pid.setTarget(&target);
   ```
> **_NOTE:_** Pointers are used with the `setConstants` and `setTarget` methods to simplify the changing of values.
3. Lastly, to acquire the controller's output, use the `calculate` method.
   ```cpp
    float output = pid.calculate(inputValue, deltaTime);
   ```

#### `MotorController`

The [`MotorController`](DroneLibrary/MotorController.h) class combines multiple [`PID`](#pid) controllers to handle the pitch, roll, and vertical velocity of the drone.

**To use the class:**
1. First, create an object by passing references to the motor power variables.
   ```cpp
    int8_t motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL;
    MotorController motorController(motorPowerTL, motorPowerTR, motorPowerBR, motorPowerBL);
   ```
2. Then, use the `setTargetValues` method to set pointers to the target values for velocity, pitch, and roll.
   ```cpp
    float targetVelocity = ..., targetPitch = ..., targetRoll = ...;
    motorController.setTargetValues(&targetVelocity, &targetPitch, &targetRoll);
   ```
3. Use the `setVelocityConstants`, `setPitchConstants`, and `setRollConstants` methods to set the PID constants for each respective controller.
   ```cpp
    PID_Instructions velocityConstants = ..., pitchConstants = ..., rollConstants = ...;
    motorController.setVelocityConstants(velocityConstants);
    motorController.setPitchConstants(pitchConstants);
    motorController.setRollConstants(rollConstants);
   ```
4. Lastly, to calculate the motor power values, use the `calculatePower` method by passing the current velocity, pitch, roll, and delta time.
   ```cpp
    motorController.calculatePower(currentVelocity, currentPitch, currentRoll, deltaTime);
   ```

#### `Orientation`

The [`Orientation`](DroneLibrary/Orientation.h) class is used to collect data from a connected `MPU60X0` gyroscope-accelerometer and process it into usable acceleration, velocity, and Euler-angle vectors.

**To use the class:**
1. First, create an object by passing the MPU address.
   ```cpp
    Orientation orientation(MPUAddress);
   ```
2. Then, use the `begin` method to initialize the MPU and calculate stationary offsets.
   ```cpp
    orientation.begin();
   ```
3. Use the `update` method to read data from the MPU and update the orientation.
   ```cpp
    orientation.update(deltaTime);
   ```
4. Finally, use the public member variables to access the calculated orientation data:
   - `angularVelocity`: degrees per second (**°/s**)
   - `angles`: Euler angles (**°**) 
   - `acceleration` and `adjustedAcceleration`: acceleration (meters per second squared, **m/s²**)
   - `velocity`: velocity (meters per second, **m/s**)

> **_NOTE:_** The `adjustedAcceleration` and velocity vectors are adjusted to fixed axes that are set when `begin` is called.

#### `RadioSendStack`

The [`RadioSendStack`](DroneLibrary/RadioSendStack.h) class is used to handle a list of messages to be sent by radio and does this by implementing a linked list structure.

**To use the class:**
1. First, create an object.
   ```cpp
    RadioSendStack sendStack;
   ```
2. Then, use the `push` or `queue` methods to add a new message to the front or back of the list, respectively.
   ```cpp
    RadioMessage message = ...;
    sendStack.push(message);
    // or
    sendStack.queue(message);
   ```
3. Use the `pop` method to collect and remove a message from the list. It can be used without arguments to collect the first element or with an index to collect a specific element.
   ```cpp
    RadioMessage message = sendStack.pop();
   ```
4. Use the `getCount` method to retrieve the total number of messages in the list.
   ```cpp
    uint8_t count = sendStack.getCount();
   ```
5. Use the `clear` method to remove all elements from the list.
   ```cpp
    sendStack.clear();
   ```

#### Custom Data Types

##### `vector3<T>`
The [`vector3<T>`](DroneLibrary/Vectors.h) struct is a 3-dimensional vector that can be made up of values of any type. It also defines basic vector opperations.

##### `SmoothValue`
The [`SmoothValue`](DroneLibrary/SmoothValue.h) class reperesents a floating-point value, but with operations that function like a lerp function, that set the new value somewhere between the current value and the desired value acording to a `smoothingFactor`. 
> **_NOTE:_** There is also a method to directly set the value without any smoothing

##### Other

There are also a few structs defined in [`RadioData.h`](DroneLibrary/RadioData.h) to simplify data handling

- `controllerInstructions` for handling controller input
```cpp
struct controllerInstructions {
    int8_t stick_X;
    int8_t stick_Y;
    int8_t power;
};
```

- `PID_Instructions` for handling PID-settings
```cpp
struct PID_Instructions {
    float k_p, k_i, k_d;
};
```

- `TargetRangeInstructions` for handling the different ranges that the drons target values can reside in
```cpp
struct TargetRangeInstructions {
    float pitchMax, rollMax, verticalVelocityMax;
};
```

- `RadioMessage` for storing a message to be sent by radio with its associated `messageType`
```cpp
struct RadioMessage {
    uint8_t messageType;
    uint8_t dataBuffer[31];
};
```

### Summery of [Drone](#drone)

The `Drone.ino` sketch is designed for an Arduino MKR Zero and integrates with various components like the MPU 6050 gyroscope/accelerometer, nRF24L01 radio device, and DarwinFPV 1104 brushless motors. The setup function initializes the radio transceiver and motor configurations, while the main loop handles radio input, flight control, and radio output. Important helper functions include `setDeltaTime` for time calculations and `sendRadio` for managing radio communications. The drone's functionality is supported by several key classes such as `PID`, `MotorController`, `Orientation`, and `RadioSendStack`, which manage tasks like PID control, motor power calculations, orientation data processing, and radio message handling. Custom data types like `vector3<T>` and `SmoothValue` are also used to facilitate data management.

## Receiver

[`Receiver.ino`](Receiver/Receiver.ino) is intended to be used with an `Arduino UNO` alongside an `nRF24L01` radio device, connected by USB to a computer that is running the [`Controller`](#controller) program.

### Basic Structure

### Important Classes

#### `InstructionHandler`

### Summery of [Receiver](#receiver)

## Controller

### Basic Structure

### Important Classes

### Summery of [Controller](#controller)

## Simulation

## Summery