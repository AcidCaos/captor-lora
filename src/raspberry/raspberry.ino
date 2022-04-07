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

void receiveEvent() {
  String message = "";
  Serial.println(" * RECV: " + message);
  while(1 < Wire.available()) {
    char c = Wire.read();
    message += c;
  }
  int x = Wire.read();    // receive byte as integer
  
  Serial.print(" * RECV: " + message);
  Serial.println("; X=" + x);
}
