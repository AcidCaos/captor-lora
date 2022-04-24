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
#define VERSION "0.4"

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

// OLED Display
#define DISP_WIDTH    128
#define DISP_HEIGHT   64
#define DISP_ADDRESS  0x3C  // I2C address

// LoRa Bands
#define ASIA_BAND   433E6
#define EUROPE_BAND 868E6
#define USA_BAND    915E6

#define BAND        EUROPE_BAND

/*
 *  CAPTOR LoRa board roles.
 * 
 *  - Node sends CAPTOR data to the gateway
 *  - Gateway receives data from the node(s)
 */

#define CAPTOR_NODE     0
#define CAPTOR_GATEWAY  1

#define CAPTOR_ROLE   CAPTOR_NODE   // <-- Modify to change board Role.

// CAPTOR PACKET CONFIG
#define CAPTOR_DELAY_REQUESTS 1500  // Time between requests
#define CAPTOR_REQUEST_PACKETS 4    // Number of packets requested to the Arduino
#define CAPTOR_PACKET_BYTES 25      // Size in bytes of each packet
#define CAPTOR_PACKET_BUFFER_N 8    // Size in number of packets of the LoRa receive packet buffer

// CAPTOR I2C ADDRESSES
#define CAPTOR_ARDUINO_ADDR 0x08    // I2C Address of the Arduino
#define CAPTOR_RASPBERRY_ADDR 0x77  // I2C Address of the Raspberry

/*
 *  DEBUG
 */

#define DISPLAY_INFO    1
#define SERIAL_LOG      1

/*
 *  BOARD MANAGER
 * 
 *  - Add the Espressif ESP32 package url https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json to the Additional Boards Manager URLs (File > Preferences)
 *  - Install esp32 by Espressif Systems. Find it on Boards Manager: http://boardsmanager#esp32
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

#if DISPLAY_INFO
SSD1306Wire display(DISP_ADDRESS, OLED_SDA, OLED_SCL);
#endif

/* SETUP */

void setup_serial();
void setup_IO_pins();
void setup_reset_init_display();
void setup_SPI_bus();
void setup_LoRa();
void setup_I2C();

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

void CAPTOR_I2C_request_and_LoRa_send(int, int, int);
void CAPTOR_check_recv_LoRa_and_I2C_send_to_RPi(int);

#endif
