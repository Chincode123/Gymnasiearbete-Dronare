// https://forum.arduino.cc/t/simple-nrf24l01-2-4ghz-transceiver-demo/405123
// SimpleRx - the slave or the receiver

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN  7
#define CSN_PIN 6

char debugBuffer[870] = {"\0"};

const byte slaveAddress[5] = {'R','x','A','A','A'};
const byte masterAddress[5] = {'T','X','a','a','a'};

RF24 radio(CE_PIN, CSN_PIN, 4000000);

char dataReceived[10]; // this must match dataToSend in the TX
int16_t replyData[2] = {109, -4000};
bool newData = false;

//===========

void setup() {

    Serial.begin(9600);
    while (!Serial);

    Serial.println("Starting");
    
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

    radio.openWritingPipe(masterAddress);
    radio.openReadingPipe(1, slaveAddress);
    radio.setRetries(3, 5);
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
    send();
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
    }
}

void send() {
    if (newData) {
        radio.stopListening();
        bool rslt = radio.write(&replyData, sizeof(replyData));
        radio.startListening();

        Serial.print("Reply Sent ");
        Serial.print(replyData[0]);
        Serial.print(", ");
        Serial.println(replyData[1]);

        if (rslt) {
            Serial.println("Ackowledge Received");
            updateReplyData();
        }
        else
            Serial.println("TX Failed");
    
        newData = false;
    }
} 

void updateReplyData() {
    replyData[0] -= 1;
    replyData[1] -= 1;

    if (replyData[0] < 100) {
        replyData[0] = 1009;
    }
    if (replyData[1] < -4009) {
        replyData[1] = -4000;
    }
}