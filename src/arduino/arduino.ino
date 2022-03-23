
#define SLAVE_ADDR  0x08 // Base direction (0x00) + jumpers (b1000)

#include <Wire.h>

/*
 Packet:
 +--+----+--------+----+--+-+-+---+
 |id| ts | AN data| nd |Te|H|C|end|
 +--+----+--------+----+--+-+-+---+
   2 bytes de ID de dispositivo                 id
   4 bytes de unix timestamp:                   ts
   8 bytes = 4 int de promedio de datos,        AN data
   4 bytes de N datos validos en cada promedio  nd
   2 bytes para temperatura: int (*10 y trunc)  Te
   1 byte  para humedad (char: -128 a 127)      H
   ======= -> 21 bytes of data to calc CRC
   1 bytes de CRC                               C
   3 bytes de fin:                              end
           [22] = 10 (byte end1)
           [23] = 13 (byte end2)
           [24] = 0; (byte end3)
*/

void requestEvent();

void setup() {

  // Init Serial
  Serial.begin(115200);

  Serial.println("Dummy Arduino");

  // Init I2C
  Wire.begin(SLAVE_ADDR);
  
  // Set Event handler
  Wire.onRequest(requestEvent);
}
 
void loop() {
  Serial.println("Waiting...");
  delay(4000);
}


int req = 0;
void requestEvent() {
  Serial.print("Requested... ");
  
  char packet[] = ".[..].dummy.packet...c...";
  
  packet[2] = (char) ((req/10) % 10) + '0';
  packet[3] = (char) ( req     % 10) + '0';

  byte CRC = 0;
  for (byte i=0; i<21; i++) CRC = CRC + packet[i];
  packet[21] = CRC;

  // packet[22] = 10; // end1 byte
  // packet[23] = 13; // end2 byte
  // packet[24] = 0;  // end3 byte

  
  Serial.println("Packet: <" + String(packet) + ">");
  Wire.print(String(packet));
  req++;
}
