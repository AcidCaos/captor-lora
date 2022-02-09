
#define SLAVE_ADDR    0x08
#define SDA           4
#define SCL           12

#include <Wire.h>

void receiveEvent(int);

void setup() {

  // Init Serial
  Serial.begin(115200);
  while (!Serial);

  Serial.println("I2C Dummy Slave");

  // Init I2C Second Peripheral
  if (!Wire1.begin(SLAVE_ADDR, SDA, SCL, -1)) {
      Serial.println("I2C could not init.");
      while(1);
  }
  
  // Set Event handler
  Wire1.onReceive(receiveEvent);
}
 
void loop() {
  delay(1000);
}

void receiveEvent(int howMany) {
  int first_byte = Wire1.read();
  Serial.println("howMany = " + String(howMany) + "; packet = " + String(packet));
}
