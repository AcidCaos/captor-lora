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

#ifndef CAPTOR_LORA_H
#define CAPTOR_LORA_H

// Version Info
#define VERSION "0.5"

/*
 *  Board PINS
 */
 
// TTGO LoRa32 V2.1 Pins
#define SCK     5   // 
#define MISO    19  // 
#define MOSI    27  // 
#define SS      18  // CS
#define DIO0    26  // Interrupt Request (IRQ)
#define RST     23  // Reset
#define DIO1    33  // 
#define BUSY    32  // 

// OLED Display Pins (First I2C Peripgeral: Wire)
#define OLED_SDA  21  // Serial Data Line
#define OLED_SCL  22  // Serial Clock Line
#define OLED_RST  16  // Reset

// I2C Pins (Second I2C Peripheral: Wire1)
#define I2C_SDA   4
#define I2C_SCL   14 // not 12, it fails to boot. not 23, it is LoRa reset.

// Other Unused Pins
#define LED       25  // LED Output Pin
#define SD_MOSI   15  // SD Card Pins
#define SD_MISO   2
#define SD_SCLK   14
#define SD_CS     13

/*
 *  Board Fixed Config
 */

// OLED Display
#define DISP_WIDTH    128
#define DISP_HEIGHT   64
#define DISP_ADDRESS  0x3C  // I2C address on Wire0

// LoRa Bands
#define ASIA_BAND   433E6
#define EUROPE_BAND 868E6
#define USA_BAND    915E6

#define BAND        EUROPE_BAND   // <-- Set used LoRa Band.

/*
 *  CAPTOR LoRa board roles.
 * 
 *  - Node sends CAPTOR data to the gateway
 *  - Gateway receives data from the node(s)
 */

#define CAPTOR_NODE     0
#define CAPTOR_GATEWAY  1

#define CAPTOR_ROLE   CAPTOR_GATEWAY   // <-- Modify to change board Role.

/*
 *  CAPTOR Parameters
 */
 
// CAPTOR PACKET CONFIG
#define CAPTOR_DELAY_REQUESTS  2    // Seconds between Arduino data requests
#define CAPTOR_REQUEST_PACKETS 1    // Number of packets requested to the Arduino
#define CAPTOR_PACKET_BYTES 25      // Size in bytes of each packet

// CAPTOR LoRa RECEIVE
#define CAPTOR_RECEIVE_DELAY   1    // Seconds between Gateway second receive buffer checks
#define CAPTOR_PACKET_BUFFER_N 8    // Size in number of packets of the LoRa receive packet buffer

/* It is recommended that: 
 * N * CAPTOR_REQUEST_PACKETS / CAPTOR_DELAY_REQUESTS < CAPTOR_PACKET_BUFFER_N / CAPTOR_RECEIVE_DELAY
 *      where N is the number of Nodes that the Gateway has.
 * Otherwise the Receive buffer will overflow
 */

// CAPTOR I2C ADDRESSES
#define CAPTOR_ARDUINO_ADDR 0x08    // I2C Address of the Arduino
#define CAPTOR_RASPBERRY_ADDR 0x77  // I2C Address of the Raspberry

/*
 *  LOW POWER mode
 *  
 *  ESP32 offers a deep sleep mode for effective power saving.
 *  In this mode CPUs, most of the RAM, and all the digital 
 *  peripherals which are clocked from APB_CLK are powered off.
 *  
 *  The only parts of the chip which can still be powered on are:
 *  RTC controller, RTC peripherals, and RTC memories.
 *  
 */

//#define LOW_POWER   // <-- Comment to disable Low Power operating mode.

#ifdef LOW_POWER // Low Power mode is only for CAPTOR Nodes.
#if CAPTOR_ROLE == CAPTOR_GATEWAY
#error "Low Power mode is only for CAPTOR Nodes. Current role is Gateway."
#endif
#endif

/*
 *  DEBUG mode
 */

#define DEBUG_MODE  // <-- Comment to disable Debug reporting mode.

#ifdef DEBUG_MODE
#define DEBUG_SERIAL(x) Serial.print(x);
#define DEBUG_SERIAL_LN(x) Serial.println(x);
#else
#define DEBUG_SERIAL(x) (void) 0;
#define DEBUG_SERIAL_LN(x) (void) 0;
#endif

/*
 *  LoRa RF parameters
 */

// Default values
//#define LORA_TX_POWER         17
//#define LORA_BANDWIDTH        125E3
//#define LORA_SPREADING_FACTOR 7
//#define LORA_CODING_RATE      5

// Output Power. LoRa library default value: 17 dBm [2..20]
#define LORA_TX_POWER         17
// Bandwidth. SX1276 default value: 125 kHz [7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3, 41.7E3, 62.5E3, 125E3, 250E3, 500E3]
#define LORA_BANDWIDTH        41.7E3
// Spreading Factor. SX1276 default value: sf=7 [6..12], SF = 2^sf
#define LORA_SPREADING_FACTOR 12
// Coding Rate. SX1276 default value: d=5 [5..8], CR = 4/d
#define LORA_CODING_RATE      8

/*
 *  LoRa Packet parameters
 */

#define LORA_PREAMBLE_LENGTH  10    // SX1276 default value: 8 [2..65535]
//#define LORA_CRC_ENABLED          // SX1276 default: disabled                                       

/*
 *  BOARD MANAGER
 * 
 *  - Add the Espressif ESP32 package url https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json 
 *    to the Additional Boards Manager URLs (File > Preferences)
 *  - Install esp32 (version 2.0.2) by Espressif Systems. Find it on Boards Manager: http://boardsmanager#esp32
 *    Version 2.0.3 breaks function setup_IO_pins() Display Reset.
 * 
 *  LIBRARIES
 * 
 *  - sandeepmistry's LoRa (version 0.8.0): https://www.arduino.cc/reference/en/libraries/lora/
 *     Find it on Library Manager: http://librarymanager#LoRa-Arduino-sending-data-radios
 *     Or on GitHub: https://github.com/sandeepmistry/arduino-LoRa
 *  - ThingPulse's OLED SSD1306 (version 4.2.1): https://www.arduino.cc/reference/en/libraries/esp8266-and-esp32-oled-driver-for-ssd1306-displays/
 *     Find it on Library Manager: http://librarymanager#I2C-display-driver-SSD1306-OLED-connected-ESP8266
 *     Or on GitHub: https://github.com/ThingPulse/esp8266-oled-ssd1306
 */

// LoRa Libraries
#include <SPI.h>
#include <LoRa.h>

// OLED Display Libraries
#include <Wire.h>
#include <SSD1306Wire.h>

#if OPERATING_MODE == DEBUG
SSD1306Wire display(DISP_ADDRESS, OLED_SDA, OLED_SCL);
#endif

/* SETUP */

void setup_serial();
void setup_IO_pins();
void setup_reset_init_display();
void setup_SPI_bus();
void setup_LoRa();
void setup_I2C();
void setup_low_power();

/* DISPLAY */

void display_clear();
void display_display();
void display_header();
void display_body();

/* LoRa */

void LoRa_receive_handler(int);
void LoRa_send(String);

/* I2C */

String I2C_request_from(int, int);
void I2C_send_to(int, String);

/* CAPTOR */

void CAPTOR_task();
void CAPTOR_loop();
void CAPTOR_I2C_request_and_LoRa_send(int, int, int);
void CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi(int);

#endif
