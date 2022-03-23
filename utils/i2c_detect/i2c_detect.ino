/*
 * MIT License - Copyright (c) 2022 by AcidCaos
 */

#include <Wire.h>

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial.println("I2C Detect");
}

void loop() {
  byte error, address, upper, lower;
  int devices = 0;
  int errors = 0;
  Serial.println("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
  for (upper = 0x0; upper <= 0x7; upper++) {
    Serial.print(String(upper) + "0:");
    for (lower = 0x0; lower <= 0xF; lower++) {
      address = (upper << 4) + lower;
      Serial.print(" ");
      if (address < 0x3 or address > 0x77) {
        Serial.print("  ");
        continue;
      }
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) { // Found Slave
        devices++;
        if (address < 16) Serial.print("0");
        Serial.print(address, HEX);
        continue;
      }
      if (error == 2) { // No device with that address
        Serial.print("--");
        continue;
      }
      switch (error) {
        case 4: Serial.print("ER"); errors++; break;  // Generic Error: Slave Mode, ...
        case 5: Serial.print("TO"); errors++; break;  // Time Out
        default: Serial.print("??"); errors++; break; // Unknown Error
      }
    }
    Serial.println("");
  }
  Serial.println("total: " + String(devices));
  if (errors > 0) Serial.println("errors: " + String(errors) + ". ER: Generic Error. TO: Time Out. ??: Unknown Error.");
  Serial.println("");
  delay(1000);
}
