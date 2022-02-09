
#define SLAVE_ADDR    0x08
#define BYTE_SEND     0x07

#include <Wire.h>
 
void setup() {

  // Init Serial
  Serial.begin(115200);
  while (!Serial);

  Serial.println("I2C Dummy Master");
}
 
void loop() {
  // Send Dummy Byte
  Wire.beginTransmission(0x8);
  Wire.write(0x09);
  Wire.endTransmission();
  Serial.print("Sent '");
  Serial.print(BYTE_SEND, HEX);
  Serial.print("' to slave @=");
  Serial.print(SLAVE_ADDR, HEX);
  Serial.print(".");
  delay(2000);
}7
