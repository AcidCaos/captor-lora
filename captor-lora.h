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
#define VERSION "0.1"

// TTGO LoRa32 V2.1 Pins
#define LED     25  // LED Output Pin
#define SCK     5   // 
#define MISO    19  // 
#define MOSI    27  // 
#define SS      18  // 
#define RST     23  // Reset
#define DIO0    26  // Interrupt Request (IRQ)

// OLED Display Pins
#define OLED_SDA  21  // Serial Data Line
#define OLED_SCL  22  // Serial Clock Line
#define OLED_RST  16  // Reset

// OLED Display
#define DISP_WIDTH    128
#define DISP_HEIGHT   64
#define DISP_ADDRESS  0x3C

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

#define CAPTOR_ROLE   CAPTOR_NODE

/*
 *  DEBUG
 */

#define DISPLAY_INFO    1
#define SERIAL_LOG      1

/*
 *  BOARD MANAGER
 * 
 *  - Add the Espressif ESP32 package url https://dl.espressif.com/dl/package_esp32_index.json to the Additional Boards Manager URLs (File > Preferences)
 *  - Install esp32 by Espressif Systems. You can find it here: http://boardsmanager#esp32
 *  
 *  LIBRARIES
 *  
 *  - sandeepmistry's LoRa (version 0.8.0): https://www.arduino.cc/reference/en/libraries/lora/ 
 *     (on GitHub: https://github.com/sandeepmistry/arduino-LoRa)
 *  - ThingPulse's OLED SSD1306 (version 4.2.1): https://www.arduino.cc/reference/en/libraries/esp8266-and-esp32-oled-driver-for-ssd1306-displays/ 
 *     (on GitHub: https://github.com/ThingPulse/esp8266-oled-ssd1306)
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

/* DISPLAY */

void display_clear();
void display_display();
void display_header();

#endif
