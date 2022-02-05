# CAPTOR LoRa
Integrating LoRa to CAPTOR nodes

## Hardware

- `TTGO LoRa32 V2.1.6` (a.k.a `TTGO T3_V1.6`, `TTGO LoRa32 V2.1 release 1.6` or `TTGO V2.1-1.6 LoRa32`)
- `Arduino Nano V3.0` (ATmega328P)

## Arduino IDE Setup

- Download and install [Arduino IDE](https://www.arduino.cc/en/software) and launch it.
- Go to `File > Preferences` and add the Espressif ESP32 package url `https://dl.espressif.com/dl/package_esp32_index.json` to the Additional Boards Manager URLs.
- Go to `Tools > Board: "Arduino Uno" > Boards Manager...` and search for `esp32`. Install [`esp32` by Espressif Systems](https://github.com/espressif/arduino-esp32#readme).
- Go to `Tools > Board: "Arduino Uno"` and under the new manager `ESP32 Arduino`, select the `TTGO LoRa32-OLED v2.1.6` board.

## Libraries

sandeepmistry [LoRa](https://www.arduino.cc/reference/en/libraries/lora/) version 0.8.0 (on [GitHub](https://github.com/sandeepmistry/arduino-LoRa))
- Library for sending and receiving data using LoRa radios. Supports Semtech SX1276 based boards/shields. 

ThingPulse [OLED SSD1306](https://www.arduino.cc/reference/en/libraries/esp8266-and-esp32-oled-driver-for-ssd1306-displays/) version 4.2.1 (on [GitHub](https://github.com/ThingPulse/esp8266-oled-ssd1306))
- I2C display driver for SSD1306 OLED displays connected to ESP32. Supports 128x64 geometry.

