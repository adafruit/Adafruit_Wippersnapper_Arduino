<p align="center">
  <img src="https://i.imgur.com/EsMTDH1.png" />
</p>

# Adafruit WipperSnapper
![Build CI](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/actions/workflows/build-clang-doxy.yml/badge.svg)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_Wippersnapper_Arduino/html/index.html)

Adafruit.io WipperSnapper is a firmware designed to turn any Wi-Fi capable board into an Internet-of-Things (IoT) device. No code required!

WipperSnapper works with [multiple microcontroller architectures](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/?tab=readme-ov-file#supported-platforms) and development boards. It is designed to be easily extensible to support new sensors, inputs, and outputs.

A **free** [Adafruit IO](https://io.adafruit.com) account is required to use WipperSnapper.

# Get Started
[Learn how to install and use WipperSnapper by following this guide on the Adafruit Learning System - QuickStart: Adafruit IO WipperSnapper](https://learn.adafruit.com/quickstart-adafruit-io-wippersnapper).


## Get WipperSnapper
Pre-compiled binaries and UF2 files for supported hardware are provided on the [releases page](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/releases) of this repository.

## Supported Platforms

|Platform| MCU(s) |
|--|--|
|[ESP32-x](https://github.com/espressif/arduino-esp32)| ESP32, ESP32-S2, ESP32-S3, ESP32-C3, ESP32-C6 |
|[ESP8266](https://github.com/esp8266/Arduino)| ESP8266 |
|[RP2040](https://github.com/earlephilhower/arduino-pico)| RP2040 MCU w/WiFi (i.e: Pico W) |
|[ATSAMD](https://github.com/adafruit/ArduinoCore-samd/)|  SAMD51 MCU w/separate WiFi Co-Processor (i.e: Adafruit "AirLift")|

## Contributing to Adafruit.io and WipperSnapper

If you have a sensor, input, or output you'd like to add Adafruit IO support for it - [we have a guide for contributing a new sensor to Adafruit.io and WipperSnapper here](https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper).

If you have a WiFi-enabled development board with a compatible microcontroller (see: "Supported Hardware" section above) and would like to add Adafruit IO support for it - [we have a guide for adding a new board to Adafruit.io and WipperSnapper here](https://learn.adafruit.com/how-to-add-a-new-board-to-wippersnapper).

## Building WipperSnapper

 -  (Preferred Method) [Build WipperSnapper with PlatformIO](https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper/build-wippersnapper-with-platformio)
 - Build WipperSnapper with Arduino
