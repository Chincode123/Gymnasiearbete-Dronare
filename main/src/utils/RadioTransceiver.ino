
bool configureRadio(RF24& radio) {
    if (!radio.setDataRate( RF24_250KBPS )) {
      Serial.println("DataRate error");
      return false;
    }
    radio.setPALevel(RF24_PA_MAX, 1);
    radio.setChannel(76);
    radio.setAutoAck(true);
    radio.setAddressWidth(5);
    radio.setPayloadSize(32);

    return true
}