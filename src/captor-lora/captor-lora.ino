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

#include <inttypes.h>

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
  CAPTOR_I2C_request_and_LoRa_send(CAPTOR_ARDUINO_ADDR, CAPTOR_PACK_REQUEST, CAPTOR_PACKET_BYTES);
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi(CAPTOR_RASPBERRY_ADDR);
  #endif

  #if DISPLAY_INFO
  display_clear();
  display_header();
  display_body();
  display_display();
  #endif

  delay(100);
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
  LoRa.onReceive(LoRa_receive_handler); // receive interrupt handler
  LoRa.receive();                       // set receive mode
  #endif
}

void setup_I2C() {
  // Init I2C Second Peripheral
  Serial.println("SETUP: Init I2C");

  #if CAPTOR_ROLE == CAPTOR_NODE || CAPTOR_ROLE == CAPTOR_GATEWAY
  if (!Wire1.begin(I2C_SDA, I2C_SCL)) { // Master Begin
    Serial.println("ERROR: I2C could not be initialized");
    while(1);
  }
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

struct {
  char packets[CAPTOR_PACKET_BUFFER_N][CAPTOR_PACKET_BYTES + 1];
  int used[CAPTOR_PACKET_BUFFER_N];
  int hint_empty;
  int hint_full;
} LoRa_buffer;

void push (String message) {
  if (LoRa_buffer.hint_full) return;
  int i = 0;
  for (i = 0; i < CAPTOR_PACKET_BUFFER_N; ++i) {
    if (not LoRa_buffer.used[i]) {
      LoRa_buffer.used[i] = 1;
      strcpy(LoRa_buffer.packets[i], (char*) message.c_str());
      LoRa_buffer.hint_empty = 0;
      return;
    }
  }
  LoRa_buffer.hint_full = 1;
  // Packet loss (buffer is full)
}

String pop () {
  if (LoRa_buffer.hint_empty) return String("");
  int i = 0;
  for (i = 0; i < CAPTOR_PACKET_BUFFER_N; ++i) {
    if (LoRa_buffer.used[i]) {
      LoRa_buffer.used[i] = 0;
      LoRa_buffer.hint_full = 0;
      return String((char *)LoRa_buffer.packets[i]);
    }
  }
  // Noting to pop
  LoRa_buffer.hint_empty = 1;
  return String("");
}

void LoRa_receive_handler (int packet_size) {
  // TTGO LoRa gateway receives a Lora packet from a TTGO LoRa node
  
  // It is not a good idea to use Serial inside an interrupt handler.
  //////// Serial.println("LoRa_receive_handler");
  
  String message = "";
  
  for (int i = 0; i < packet_size; i++) {
    message += (char)LoRa.read();
  }
  
  // It's also not a good idea to send **now** the message to the 
  // Raspberry Pi through I2C from an interrupt handler.
  
  // So, we save it into a very simple buffer that will eventually be 
  // read and sent safely through I2C to the Raspberry.

  push (message);

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
  count_send_num++;
}

/*
void LoRa_send_dummy() {
  String data = String(count_send_num);
  count_send_num++;
  LoRa_send(data);
}
*/

/*
 * GATEWAY I2C
 */

void I2C_send_to(int address, String packet) {
  // The TTGO LoRa Gateway forwards a packet (through I2C) to address.
  Serial.println("I2C_send_to_RPi" + packet + " (" + packet.length() + " bytes)");
  Wire1.beginTransmission(address);
  Wire1.write((char*) packet.c_str());
  Wire1.endTransmission();
}

/*
uint32_t req = 0;
void I2C_request_handler() {
  Serial.println("I2C_request_handler");
  Wire1.print(req);
  Wire1.print(" Packets.");
  Serial.print(" * SENT: " + String(req) + " Packets.");
  req++;
}
*/

/*
 * NODE I2C
 */

String I2C_request_from(int slave, int bytes) {
  Serial.println("I2C_request_from");
  int avail = Wire1.requestFrom(slave, bytes);
  if (avail > 0) {
    Serial.println(" * Available data. " + String(avail) + " bytes.");
    char buffer[avail];
    Wire1.readBytes(buffer, avail);
    buffer[avail] = 0; // End of string
    String convert(buffer);
    Serial.println(" * Request response: " + convert);
    return convert;
  }
  return String("");
}

/*
 * NODE CAPTOR
 */

void CAPTOR_I2C_request_and_LoRa_send(int slave, int num_packets, int bytes) {
  // A TTGO LoRa node reads data (through I2C) from the Arduino (slave with sensors),
  // and sends it to the TTGO LoRa Gateway.
  Serial.println("CAPTOR_I2C_request_and_LoRa_send");
  for (int i = 0; i < num_packets; i++){
    
    String p_i = I2C_request_from(slave, bytes);

    if (p_i != "") LoRa_send(String(p_i));
  }
}

/*
 * GATEWAY CAPTOR
 */

void CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi (int rpi_address) {
  Serial.println("CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi");
  String message = pop();
  while (message != "") {
    Serial.println(" * Forward packet to RPi: " + message  + " (" + message.length() + " bytes)");
    last_message_recv = message;

    // To avoid a long loop, we let the RT-OS do its housekeeping and then it
    // returns here. Otherwise, we hay have problems with the Interrupt Watchdog
    // <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html#interrupt-watchdog>
    yield();

    // The TTGO LoRa Gateway forwards a packet (through I2C) to the Raspberry.
    I2C_send_to(rpi_address, message);
    
    message = pop();
  }
}
