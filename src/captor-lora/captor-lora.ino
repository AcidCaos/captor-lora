/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 by AcidCaos
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "captor-lora.h"

void setup() {
  
  setup_serial();
  setup_IO_pins();
  setup_reset_init_display();
  setup_SPI_bus();
  setup_LoRa();
  setup_I2C();
  
}

void loop() {

  #if CAPTOR_ROLE == CAPTOR_NODE
  //LoRa_send_dummy();
  CAPTOR_I2C_request_and_LoRa_send(CAPTOR_ARDUINO_ADDR, CAPTOR_PACK_REQUEST, CAPTOR_PACKET_BYTES);
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  #endif

  #if DISPLAY_INFO
  display_clear();
  display_header();
  display_body();
  display_display();
  #endif

  delay(1500);
}

/*
 * SETUP
 */

void setup_serial() {
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println();
}

void setup_IO_pins() {
  // Set IO Pins
  Serial.println("SETUP: Set IO Pins");
  pinMode(OLED_RST, OUTPUT); // Display Reset
}

void setup_reset_init_display() {
  // Reset Display
  Serial.println("SETUP: Reset Display");
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  
  // Init Display
  Serial.println("SETUP: Init Display");
  display.init();
  display.setContrast(127); // 0..255
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  delay(1500);
}

void setup_SPI_bus() {
  // Init SPI bus 
  Serial.println("SETUP: Init SPI bus");
  SPI.begin(SCK, MISO, MOSI, SS);
}

void setup_LoRa() {
  // Init LoRa
  Serial.println("SETUP: Init LoRa");
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(BAND)) {
    Serial.println("ERROR: LoRa could not be initialized");
    while (1);
  }
  Serial.println("SETUP: LoRa Band: " + String(BAND) + "Hz");
  delay(1500);
  
  #if CAPTOR_ROLE == CAPTOR_NODE
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
  LoRa.onReceive(LoRa_receive_handler); // receive interrupt handler
  #endif
}

void setup_I2C() {
  // Init I2C Second Peripheral
  Serial.println("SETUP: Init I2C");

  #if CAPTOR_ROLE == CAPTOR_NODE
  if (!Wire1.begin(I2C_SDA, I2C_SCL)) { // Master Begin
    Serial.println("ERROR: I2C could not be initialized");
    while(1);
  }
  #endif
  
  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  if (!Wire1.begin(CAPTOR_GATEWAY_ADDR, I2C_SDA, I2C_SCL, 0)) { // Slave Begin
    Serial.println("ERROR: I2C could not be initialized");
    while(1);
  }
  // Set Slave Event handlers
  Wire1.onReceive(I2C_receive_handler);
  Wire1.onRequest(I2C_request_handler);
  // Pre-write to the slave response buffer. (ESP32 only! More at: https://github.com/espressif/arduino-esp32/blob/master/docs/source/api/i2c.rst)
  char message[64];
  sprintf(message, 64, "%u Packets.", req++);
  Wire1.slaveWrite((const uint8_t) message, strlen(message));
  #endif
}

/*
 * DISPLAY
 */

void display_clear() {
  display.clear();
}
  
void display_display() {
  display.display();
}

void display_header() {
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_16);  
  display.drawString(0, 0, "CAPTOR");
  display.setFont(ArialMT_Plain_10);
  #if CAPTOR_ROLE == CAPTOR_NODE
  display.drawString(68, 5, "node v" + String(VERSION));
  #endif
  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  display.drawString(68, 5, "gateway v" + String(VERSION));
  #endif
}

int count_send_num = 0;
String last_message_recv = "";
void display_body() {
  #if CAPTOR_ROLE == CAPTOR_NODE
  display.drawString(0, 13, "sent: " + String(count_send_num));
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  display.drawString(0, 13, "last: " + last_message_recv);
  #endif
}

/*
 * GATEWAY LoRa
 */

void LoRa_receive_handler (int packet_size) {
  Serial.print("LoRa_receive_handler");
  String message = "";
  while (LoRa.available()) {
    message += (char)LoRa.read();
  }
  Serial.println(" * RECV: " + message);
  last_message_recv = message;
}

void LoRa_request_handler () {
  
}

/*
 * NODE LoRa
 */

void LoRa_send(String message) {
  Serial.println("LoRa_send");
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket(); // endPacket(true) => async (non-blocking mode)
  Serial.println(" * SENT: " + message);
  
}

void LoRa_send_dummy() {
  String data = String(count_send_num);
  count_send_num++;
  LoRa_send(data);
}

/*
 * GATEWAY I2C
 */

void I2C_receive_handler(int howMany) {
  Serial.print("I2C_receive_handler");
  int first_byte = Wire1.read();
  Serial.print(" * RECV: howMany = " + String(howMany) + "; byte[0] = ");
  Serial.println(first_byte, HEX);
}

uint32_t req = 0;
void I2C_request_handler() {
  Serial.print("I2C_request_handler");
  Wire1.print(req);
  Wire1.print(" Packets.");
  Serial.print(" * SENT: " + String(req) + " Packets.");
  req++;
}

/*
 * NODE I2C
 */

String I2C_request_from(int slave, int bytes) {
  Serial.println("I2C_request_from");
  int avail = Wire1.requestFrom(slave, bytes);
  if (avail > 0) {
    Serial.println("Available data. " + String(avail) + " bytes.");
    char buffer[avail];
    Wire1.readBytes(buffer, avail);
    buffer[avail] = 0; // End of string
    String convert(buffer);
    Serial.println("Request response: " + convert);
    return convert;
  }
  return String("");
}

/*
 * CAPTOR
 */

void CAPTOR_I2C_request_and_LoRa_send(int slave, int num_packets, int bytes) {
  for (int i = 0; i < num_packets; i++){
    
    String p_i = I2C_request_from(slave, bytes);
    //String p_i = "shit " + String(i);

    if (p_i != "") {
      //LoRa_send_dummy();
      LoRa_send(String(p_i));
    }
  }
}
