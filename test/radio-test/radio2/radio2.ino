// SimpleRx - the slave or the receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN  7
#define CSN_PIN 6

char debugBuffer[870] = {"\0"};

const byte thisSlaveAddress[5] = {'R','x','A','A','A'};

RF24 radio(CE_PIN, CSN_PIN, 4000000);

char dataReceived[10]; // this must match dataToSend in the TX
bool newData = false;

//===========

void setup() {

    Serial.begin(9600);
    while (!Serial);

    Serial.println("SimpleRx Starting");
    
    if (!radio.begin()){
        Serial.println(F("radio hardware not responding!"));
        while (true);
    }

    if (!radio.setDataRate( RF24_250KBPS )) {
      Serial.println("DataRate error");
      Serial.println(radio.getDataRate());
      while (true);
    }

    radio.setPALevel(RF24_PA_MAX, 1);

    radio.openReadingPipe(1, thisSlaveAddress);
    radio.startListening();

    uint16_t usedChars = radio.sprintfPrettyDetails(debugBuffer);
    Serial.println(debugBuffer);
    Serial.print(F("strlen = "));
    Serial.println(usedChars + 1);
    while (true) {
      bool exit = false;
      while (Serial.available()) {
        if (Serial.read() == 10)
          exit = true;
      }
      if (exit)
        break;
    }
}

//=============

void loop() {
    getData();
    showData();
}

//==============

void getData() {
    if ( radio.available() ) {
        radio.read( &dataReceived, sizeof(dataReceived) );
        newData = true;
    }
}

void showData() {
    if (newData == true) {
        Serial.print("Data received ");
        Serial.println(dataReceived);
        newData = false;
    }
}