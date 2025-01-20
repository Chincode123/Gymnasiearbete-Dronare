#include <Wire.h>

const int MPU = 0x68;

float gyroX = 0, gyroY = 0, gyroZ = 0;
float accX = 0, accY = 0, accZ = 0;

float rollG = 0, pitchG = 0;

float rollComp = 0, pitchComp = 0;

float errorR = 0, errorP = 0;

float pitchOffset = 0, rollOffset = 0;

float deltaTime = 0;
float previousTime = 0;

void setup() {
    Serial.begin(9600);
    while (!Serial);

    Wire.begin();
    Wire.beginTransmission(MPU);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    previousTime = millis();
}

void loop() {
    deltaTime = (millis() - previousTime) * 0.001;

    getAccel();
    getGyro();

    float rollA = atan(accX / accZ) / 2 / PI * 360;
    float pitchA = atan(accY / accZ) / 2 / PI * 360;

    rollComp = rollA * 0.005 + 0.995 * ((rollComp + rollOffset) + gyroY * deltaTime) + errorR * 0.005;
    pitchComp = pitchA * 0.005 + 0.995 * ((pitchComp + pitchOffset) + gyroX * deltaTime) + errorP * 0.005;

    errorP = errorP + (pitchA - pitchComp) * deltaTime;
    errorR = errorR + (rollA - rollComp) * deltaTime;

    while (Serial.available()) {
        if (Serial.read() == 10) {
            rollOffset = rollComp;
            pitchOffset = pitchComp;
        }
    }

    rollComp = rollComp - rollOffset;
    pitchComp = pitchComp - pitchOffset;

    Serial.print(pitchComp);
    Serial.print("/");
    Serial.println(rollComp);

    previousTime = millis();
}

void getAccel() {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    accX = (Wire.read() << 8 | Wire.read()) / 163840.0;
    accY = (Wire.read() << 8 | Wire.read()) / 163840.0;
    accZ = (Wire.read() << 8 | Wire.read()) / 163840.0;
}

void getGyro() {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    gyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    gyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}