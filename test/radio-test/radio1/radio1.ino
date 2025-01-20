// SimpleTx - the master or the transmitter

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


#define CE_PIN   7
#define CSN_PIN  8

const byte slaveAddress[5] = {'R','x','A','A','A'};

char debugBuffer[870] = {"\0"};

RF24 radio(CE_PIN, CSN_PIN, 4000000); // Create a Radio

char dataToSend[10] = "Message 0";
char txNum = '0';


unsigned long currentMillis;
unsigned long prevMillis;
unsigned long txIntervalMillis = 1000; // send once per second


void setup() {

    Serial.begin(9600);
    while (!Serial);

    Serial.println("SimpleTx Starting");

    if (!radio.begin()){
        Serial.println(F("radio hardware not responding!"));
        while (true);
    }

    if (!radio.setDataRate( RF24_250KBPS )) {
      Serial.println("DataRate error");
      Serial.print("data rate: ");
      Serial.println(radio.getDataRate());
      while (true);
    }
    
    radio.setPALevel(RF24_PA_MAX, 1);

    radio.setRetries(3,5); // delay, count
    radio.openWritingPipe(slaveAddress);

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

//====================

void loop() {
    currentMillis = millis();
    if (currentMillis - prevMillis >= txIntervalMillis) {
        send();
        prevMillis = millis();
    }
}

//====================

void send() {

    bool rslt;
    rslt = radio.write( &dataToSend, sizeof(dataToSend) );
        // Always use sizeof() as it gives the size as the number of bytes.
        // For example if dataToSend was an int sizeof() would correctly return 2

    Serial.print("Data Sent ");
    Serial.print(dataToSend);
    if (rslt) {
        Serial.println("  Acknowledge received");
        updateMessage();
    }
    else {
        Serial.println("  Tx failed");
    }
}

//================

void updateMessage() {
        // so you can see that new data is being sent
    txNum += 1;
    if (txNum > '9') {
        txNum = '0';
    }
    dataToSend[8] = txNum;
}