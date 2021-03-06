# CAPTOR LoRa
Integrating LoRa to CAPTOR nodes

## Hardware

- `TTGO LoRa32 V2.1.6` (a.k.a `TTGO T3_V1.6`, `TTGO LoRa32 V2.1 release 1.6` or `TTGO LoRa32 V2.1-1.6`), revision 1.6.1 also compatible.
- `Arduino Nano V3.0` (ATmega328P)
- `Raspberry Pi 3 Model B V1.2`

## Arduino IDE Setup

- Download and install [Arduino IDE](https://www.arduino.cc/en/software) and launch it.
- Go to `File > Preferences` and add the Espressif ESP32 package url `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json` to the Additional Boards Manager URLs.
  - Note: For compatibility with I2C slave mode, Espressif package index (`https://dl.espressif.com/dl/package_esp32_index.json`) should not be used, since it only contains old versions (up to 1.6.1). I2C slave support [was added](https://github.com/espressif/arduino-esp32/pull/5746) on version [2.0.1](https://github.com/espressif/arduino-esp32/releases/tag/2.0.1).
- Go to `Tools > Board > Boards Manager`, search for `esp32` and install the one by [Espressif Systems](https://github.com/espressif/arduino-esp32#readme), version 2.0.1 or 2.0.2. Version 2.0.3 breaks function setup_IO_pins() Display Reset.
- Go to `Tools > Board` and under the new manager `ESP32 Arduino`, select the `TTGO LoRa32-OLED` board.
- Go to `Tools > Board Revision: "TTGO LoRa32 V1 (No TFCard)"` and select the `TTGO LoRa32 V2.1 (1.6.1)` board revision.

## Libraries

sandeepmistry [LoRa](https://www.arduino.cc/reference/en/libraries/lora/) version 0.8.0 (on [GitHub](https://github.com/sandeepmistry/arduino-LoRa))
- Library for sending and receiving data using LoRa radios. Supports Semtech SX1276 based boards/shields. 

ThingPulse [OLED SSD1306](https://www.arduino.cc/reference/en/libraries/esp8266-and-esp32-oled-driver-for-ssd1306-displays/) version 4.2.1 (on [GitHub](https://github.com/ThingPulse/esp8266-oled-ssd1306))
- I2C display driver for SSD1306 OLED displays connected to ESP32. Supports 128x64 geometry.

