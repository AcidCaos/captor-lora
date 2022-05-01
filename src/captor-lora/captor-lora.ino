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
#include <Esp.h> // For Low Power and ESP32 data <https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/Esp.h>

#include "captor-lora.h"

void setup() {

  setup_serial();
  setup_IO_pins();
  setup_reset_init_display();
  setup_SPI_bus();
  setup_LoRa();
  setup_I2C();
  setup_low_power();
  
}

void loop() {
  #ifndef LOW_POWER // In Low Power mode, loop is never reached.
  CAPTOR_loop();
  #endif
}

/*
 * SETUP
 */

void setup_serial() {
  #ifdef DEBUG_MODE
  // Init Serial Monitor
  Serial.begin(115200);
  Serial.println();
  #endif
}

void setup_IO_pins() {
  // Set IO Pins
  DEBUG_SERIAL_LN("SETUP: Set IO Pins");
  pinMode(OLED_RST, OUTPUT); // Display Reset
}

void setup_reset_init_display() {
  #ifdef DEBUG_MODE
  // Reset Display
  DEBUG_SERIAL_LN("SETUP: Reset Display");
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  
  // Init Display
  DEBUG_SERIAL_LN("SETUP: Init Display");
  display.init();
  display.setContrast(127); // 0..255
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  delay(1500);
  #endif
}

void setup_SPI_bus() {
  // Init SPI bus 
  DEBUG_SERIAL_LN("SETUP: Init SPI bus");
  SPI.begin(SCK, MISO, MOSI, SS);
}

void setup_LoRa() {
  // Init LoRa
  DEBUG_SERIAL_LN("SETUP: Init LoRa");
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(BAND)) {
    DEBUG_SERIAL_LN("ERROR: LoRa could not be initialized");
    while (1);
  }
  DEBUG_SERIAL_LN("SETUP: LoRa Band: " + String(BAND) + "Hz");
  delay(1500);

  // LoRa Radio parameters config
  LoRa.setTxPower(LORA_TX_POWER);                 // Output Power = 17 dBm 
                                                  // * LoRa library default value: 17 dBm
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);        // Bandwidth = 125 kHz
                                                  // * SX1276 default value: 125 kHz
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR); // Spreading Factor = 2^sf = 128 chips/symbol
                                                  // * SX1276 default value: sf=7 SF = 128
  LoRa.setCodingRate4(LORA_CODING_RATE);          // Coding Rate = 4/d = 4/5
                                                  // * SX1276 default value: d=5, CR = 4/5
  // LoRa Packet parameters config
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH);   // Preamble Length between 2 and 65535
                                                  // * SX1276 default value: 8
  #ifdef LORA_CRC_ENABLED
  LoRa.enableCrc();                               // Payload CRC = enabled. Or: disableCrc()
                                                  // * SX1276 default value: disabled
  #endif
  
  // Set operation mode
  #if CAPTOR_ROLE == CAPTOR_NODE
  LoRa.idle();                          // set standby mode
  LoRa.disableInvertIQ();               // normal mode
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  LoRa.disableInvertIQ();               // normal mode
  LoRa.onReceive(LoRa_receive_handler); // receive interrupt handler
                                        // Note: If using implicit mode when sending,
                                        //   packets must be received with explicit size:
                                        //   receive(int size) [or parsePacket(int size)]
  LoRa.receive();                       // set receive mode
  #endif
}

void setup_I2C() {
  // Init I2C Second Peripheral
  DEBUG_SERIAL_LN("SETUP: Init I2C");

  #if CAPTOR_ROLE == CAPTOR_NODE || CAPTOR_ROLE == CAPTOR_GATEWAY
  if (!Wire1.begin(I2C_SDA, I2C_SCL)) { // Master Begin
    DEBUG_SERIAL_LN("ERROR: I2C could not be initialized");
    while(1);
  }
  #endif
}

#ifdef LOW_POWER
// store data in RTC memory to use it over reboots
RTC_DATA_ATTR int bootCount = 0;
#endif

void setup_low_power() {
  // Init Low Power on reboot
  #ifdef LOW_POWER
  DEBUG_SERIAL_LN("SETUP: Init Low Power");
  
  // Increment boot number and print it every reboot
  ++bootCount;
  DEBUG_SERIAL_LN(" * Boot count: " + String(bootCount));

  // Print wake up reason
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    DEBUG_SERIAL_LN(" * Wakeup caused by timer");
  } else {
    DEBUG_SERIAL_LN(" * Wakeup caused by external signal (RTC_IO or RTC_CNTL),")
    DEBUG_SERIAL_LN("   touchpad, ULP program, or was not caused by deep sleep.");
  }
  
  // Do what is supposed to be done (CAPTOR Node task)
  CAPTOR_task();
  
  // Set up (again) the wakeup source
  esp_sleep_enable_timer_wakeup(CAPTOR_DELAY_REQUESTS * 1000000ULL); // microseconds
  DEBUG_SERIAL_LN(" * Set next sleep for " + String(CAPTOR_DELAY_REQUESTS) + "sec.");

  // Enter Deep Sleep <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html>
  esp_deep_sleep_start();
  // this will never be reached.
  #endif
}

/*
 * DISPLAY
 */

#ifdef DEBUG_MODE
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

struct {
  int count = 0;
  int err = 0;
  String message; // char packet[CAPTOR_PACKET_BYTES + 1];
  int size = 0; // bytes
  int rssi = 0; // dBm
  float snr = 0; // dB
  long frequencyError = 0; // Hz
} Last_packet;

void display_body() {
  #if CAPTOR_ROLE == CAPTOR_NODE
  display.drawString(0, 13, "sent: " + String(count_send_num));
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  
  Last_packet.rssi = LoRa.packetRssi();
  Last_packet.snr = LoRa.packetSnr();
  Last_packet.frequencyError = LoRa.packetFrequencyError();
  
  display.drawString(0, 13, "last: " + Last_packet.message);
  display.drawString(0, 24, "size: " + String(Last_packet.size) + " bytes");
  display.drawString(0, 35, "rssi: " + String(Last_packet.rssi) + " dBm");
  display.drawString(0, 46, "snr: " + String(Last_packet.snr) + " dB");
  
  display.drawString(64, 24, "fe: " + String(Last_packet.frequencyError) + " Hz");
  display.drawString(64, 35, "count: " + String(Last_packet.count));
  display.drawString(64, 46, "err: " + String(Last_packet.err));
  #endif
}
#endif

/*
 * GATEWAY LoRa
 */

struct { // Really simple Receive buffer
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
  
  String message = "";
  
  for (int i = 0; i < packet_size; i++) {
    message += (char)LoRa.read();
  }
  
  // Check CRC
  byte CRC = 0;
  for (byte i=0; i<21; i++) CRC = CRC + (char) message[i];
  if (message[21] != CRC) { // Invalid packet
    #ifdef DEBUG_MODE
    Last_packet.err ++;
    #endif
    return;
  }
  
  // It's also not a good idea to send **now** the message to the 
  // Raspberry Pi through I2C from an interrupt handler.
  // So, we save it into a very simple buffer that will eventually be 
  // read and sent safely through I2C to the Raspberry.

  #ifdef DEBUG_MODE
  Last_packet.message = message;
  Last_packet.size = packet_size;
  Last_packet.count ++;
  #endif

  push (message);

}

/*
 * NODE LoRa
 */

void LoRa_send(String message) {
  DEBUG_SERIAL("NODE [LoRa] SEND " + String(message.length()) + " bytes... ");
  LoRa.beginPacket();
  LoRa.print(message);
  int ret = LoRa.endPacket(); // endPacket(true) => async (non-blocking mode)
  if (ret) {DEBUG_SERIAL_LN("Ok.");}
  else {DEBUG_SERIAL_LN("/!\\ FAILED");}
  #ifdef DEBUG_MODE
  count_send_num++;
  #endif
}

/*
 * GATEWAY I2C
 */

void I2C_send_to(int address, String packet) {
  // The TTGO LoRa Gateway forwards a packet (through I2C) to address.
  DEBUG_SERIAL_LN("GATEWAY [I2C] SEND_TO @=" + String(address) + "; (" + String(packet.length()) + " bytes): <" + packet + ">");
  Wire1.beginTransmission(address);
  Wire1.write((char*) packet.c_str());
  Wire1.endTransmission();
}

/*
 * NODE I2C
 */

String I2C_request_from(int slave, int bytes) {
  DEBUG_SERIAL("NODE [I2C] REQUEST_FROM @=" + String(slave) + ", " + String(bytes) + " bytes... ");
  int avail = Wire1.requestFrom(slave, bytes);
  if (avail > 0) {
    DEBUG_SERIAL("GOT " + String(avail) + " bytes. ");
    char buffer[avail];
    Wire1.readBytes(buffer, avail);
    buffer[avail] = 0; // End of string
    String convert(buffer);
    DEBUG_SERIAL_LN("RESPONSE: <" + convert + ">");
    return convert;
  } else DEBUG_SERIAL_LN("/!\\ FAILED");
  return String("");
}

/*
 * CAPTOR
 */

void CAPTOR_loop () {
  #if CAPTOR_ROLE == CAPTOR_NODE
  CAPTOR_I2C_request_and_LoRa_send(CAPTOR_ARDUINO_ADDR, CAPTOR_REQUEST_PACKETS, CAPTOR_PACKET_BYTES);
  #endif

  #if CAPTOR_ROLE == CAPTOR_GATEWAY
  CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi(CAPTOR_RASPBERRY_ADDR);
  #endif

  #ifdef DEBUG_MODE
  display_clear();
  display_header();
  display_body();
  display_display();
  #endif

  #if CAPTOR_ROLE == CAPTOR_NODE
  delay(CAPTOR_DELAY_REQUESTS * 1000); // milliseconds
  #elif CAPTOR_ROLE == CAPTOR_GATEWAY
  delay(CAPTOR_RECEIVE_DELAY * 1000); // milliseconds
  #endif
}

/*
 * NODE CAPTOR
 */

void CAPTOR_I2C_request_and_LoRa_send(int slave, int num_packets, int bytes) {
  // A TTGO LoRa node reads data (through I2C) from the Arduino (slave with sensors),
  // and sends it to the TTGO LoRa Gateway.
  for (int i = 0; i < num_packets; i++){
    
    String p_i = I2C_request_from(slave, bytes);

    if (p_i != "") LoRa_send(String(p_i));
  }
}

/*
 * GATEWAY CAPTOR
 */

void CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi (int rpi_address) {
  String message = pop();
  while (message != "") {
    DEBUG_SERIAL_LN("GATEWAY [LoRa] RECEIVE (" + String(message.length()) + " bytes): <" + message + ">");

    // To avoid a long loop, we let the RT-OS do its housekeeping and then it
    // returns here. Otherwise, we may have problems with the Interrupt Watchdog
    // <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/wdts.html#interrupt-watchdog>
    yield();

    // The TTGO LoRa Gateway forwards a packet (through I2C) to the Raspberry.
    I2C_send_to(rpi_address, message);
    
    message = pop();
  }
}
