
#define SLAVE_ADDR    0x08

#include <Wire.h>

// void receiveEvent(int);
void requestEvent();



void setup() {

  // Init Serial
  Serial.begin(115200);

  Serial.println("Dummy Arduino");

  // Init I2C
  Wire.begin(SLAVE_ADDR);
  
  // Set Event handler
  // Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
}
 
void loop() {
  Serial.println("Waiting...");
  delay(4000);
}

/*
void receiveEvent(int howMany) {
  Serial.print("Receiving... ");
  int first_byte = Wire.read();
  Serial.print("howMany = " + String(howMany) + "; byte[0] = ");
  Serial.println(first_byte, HEX);
}
*/

int req = 0;
void requestEvent() {
  Serial.print("Requested... ");
  char packet[] = "..................dummy..";
  packet[4] = (char) (req % 10) + '0';
  Serial.println("Packet: <" + String(packet) + ">");
  Wire.print(String(packet));
  req++;
}
