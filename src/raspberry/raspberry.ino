/*
This is an Arduino implementation of what the Raspberry Pi should do (a stub).
*/

#define CAPTOR_RASPBERRY_ADDR 0x99

#include <Wire.h>

void receiveEvent();

void setup() {

  // Init Serial
  Serial.begin(115200);

  Serial.println("Gateway I2C Slave");

  // I2C
  Wire.begin(CAPTOR_RASPBERRY_ADDR);
  Wire.onReceive(receiveEvent);
}

void loop() {
  Serial.println("Waiting...");
  delay(2000);
}

void receiveEvent(int packet_size) {
  String message = "";
  for (int i = 0; i < packet_size; i++) {
    message += (char)Wire.read();
  }
  Serial.println(" * RECV: " + message + " (" + message.length() + " bytes)");
}
