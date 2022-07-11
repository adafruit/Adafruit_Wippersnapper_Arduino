# Adafruit IO - WipperSnapper Beta ![Build CI](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/actions/workflows/build-clang-doxy.yml/badge.svg)[![Documentation](https://github.com/adafruit/ci-arduino/blob/master/assets/doxygen_badge.svg)](http://adafruit.github.io/Adafruit_Wippersnapper_Arduino/html/index.html)


Adafruit.io allows you to monitor, interact with, and share your project's data in minutes.

Adafruit.io WipperSnapper is the firmware designed to turn any Wi-Fi capable board into an Internet-of-Things (IoT) device. No code required!

WipperSnapper works with multiple microcontroller architectures such as ESP8266, ESP32, ESP32-S2, ESP32-C3, and ATSAMD51.

You will need a **free** [Adafruit IO](https://io.adafruit.com) account to use WipperSnapper.

**This software is in beta** and is actively being developed. Please report bugs via the [Adafruit IO Support Page](https://io.adafruit.com/support).

# Get Started
[Learn how to install and use WipperSnapper by following this guide on the Adafruit Learning System - QuickStart: Adafruit IO WipperSnapper](https://learn.adafruit.com/quickstart-adafruit-io-wippersnapper).


## Get WipperSnapper
Pre-compiled binaries and UF2 files for supported hardware are provided on the [releases page](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/releases) of this repository.

## Supported Hardware
The following microcontrollers are supported by Adafruit WipperSnapper:
* Espressif ESP8266
* Espressif ESP32
* Espressif ESP32-S2
* Espressif ESP32-S3
* Espressif ESP32-C3
* Microchip ATSAMD51 + [AirLift WiFi Co-Processor](https://www.adafruit.com/?q=airlift+wifi&sort=BestMatch)

## Contributing to Adafruit.io and WipperSnapper

If you have a sensor, input, or output you'd like to add Adafruit IO support for it - [we have a guide for contributing a new sensor to Adafruit.io and WipperSnapper here](https://learn.adafruit.com/how-to-add-a-new-component-to-adafruit-io-wippersnapper).

If you have a WiFi-enabled development board with a compatible microcontroller (see: "Supported Hardware" section above) and would like to add Adafruit IO support for it - [we have a guide for adding a new board to Adafruit.io and WipperSnapper here](https://learn.adafruit.com/how-to-add-a-new-board-to-wippersnapper).

## Building WipperSnapper with Arduino
If you would like to build and develop WipperSnapper locally, [we suggest following the steps presented on the guide on this page](https://learn.adafruit.com/how-to-add-a-new-board-to-wippersnapper/build-wippersnapper).

## (BETA) Building WipperSnapper with PlatformIO
We support building and locally testing WipperSnapper with the Platform IO IDE for Visual Studio Code. These instructions are subject to modification.

### Platform IO Build Instructions
* Download and install [Microsoft Visual Studio Code](https://code.visualstudio.com), PlatformIO IDE is built on top of it
* Follow the instructions on [platformio's website to install the Platform IO IDE extension](https://platformio.org/install/ide?install=vscode).
* [Make a local clone](https://docs.github.com/en/repositories/creating-and-managing-repositories/cloning-a-repository) of the [Adafruit WipperSnapper Arduino repository](https://github.com/adafruit/Adafruit_Wippersnapper_Arduino) on your computer. 
* In `platformio.ini` file in the root of this repository, find the board you'd like to build WipperSnapper for. Change its `upload_port` and `monitor_port` to match your system.
* The `platformio.ini` file in the root of this repository includes multiple environments to build WipperSnapper for different boards. In the [Platform IO toolbar](https://docs.platformio.org/en/stable//integration/ide/vscode.html#platformio-toolbar), use the Project environment switcher to switch to the board you'd like to upload to.
* Open the `examples/Wippersnapper_demo/Wippersnapper_demo.ino` file.
* On the [Platform IO toolbar](https://docs.platformio.org/en/stable//integration/ide/vscode.html#platformio-toolbar), click the build (checkmark) button. If there are no build errors, click the upload (right arrow) button.
* WipperSnapper should be uploaded to your board. You may use the PlatformIO Serial Monitor to view WipperSnapper's debug output.