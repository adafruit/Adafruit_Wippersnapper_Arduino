/*!
 * @file src/components/uart/serial_config.h
 *
 * Serial configuration definitions for WipperSnapper components.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_UART_SERIAL_CONFIG_H
#define WS_UART_SERIAL_CONFIG_H

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_SAMD) ||              \
    defined(ARDUINO_ARCH_ESP8266)
// SAMD supports native Arduino SoftwareSerial API
// ESP8266 supports SoftwareSerial in the ESP8266 Arduino core
// RP2040/RP2350 supports a wrapper around SoftwareSerial and emulation via
// PIOUART (see:
// https://arduino-pico.readthedocs.io/en/latest/piouart.html#softwareserial-emulation)
#include <SoftwareSerial.h>
#define HAS_SW_SERIAL 1 ///< Indicates that the board supports SoftwareSerial
#else
#define HAS_SW_SERIAL                                                          \
  0 ///< Indicates that the board DOES NOT support SoftwareSerial
#endif

#endif // WS_UART_SERIAL_CONFIG_H