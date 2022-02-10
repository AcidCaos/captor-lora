
#define SLAVE_ADDR    0x08
#define BYTE_SEND     0x03

#include <Wire.h>
 
void setup() {

  // Init Serial
  Serial.begin(115200);
  while (!Serial);

  Serial.println("I2C Dummy Master");

  // I2C
  Wire.begin();
}
 
void loop() {
  int ret;
  // Send Dummy Byte
  Serial.print("Sending... ");
  Wire.beginTransmission(SLAVE_ADDR);
  ret = Wire.endTransmission();
  if (ret == 2) { // No Slave on SLAVE_ADDR
    Serial.print("No slave in @=");
    Serial.println(SLAVE_ADDR, HEX);
  }
  else if (ret == 0) { // Slave Available
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write(BYTE_SEND);
    Wire.endTransmission();
    Serial.print("Sent byte 0x");
    Serial.print(BYTE_SEND, HEX);
    Serial.print(" to slave @=");
    Serial.print(SLAVE_ADDR, HEX);
    Serial.println(".");
  }
  else {
    Serial.print("ERROR occurred when sending byte 0x");
    Serial.print(BYTE_SEND, HEX);
    Serial.print(" to slave @=");
    Serial.print(SLAVE_ADDR, HEX);
    Serial.println(".");
  }
  delay(1000);
}
