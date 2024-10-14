#include <Firmata.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include ".\RadioData.h"

RF24 radio(/*SE pin*/, /*CSN pin*/)

void setup() {
    radio.begin();
    radio.openWritingPipe(PIPE_ADDRESSES[0]);
    radio.openReadingPipe(1, PIPE_ADDRESSES[1]);
}

void loop() {

}