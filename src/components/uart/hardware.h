/*!
 * @file src/components/uart/hardware.h
 *
 * Low-level hardware implementation for WipperSnapper's uart component.
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
#ifndef WS_UART_HARDWARE_H
#define WS_UART_HARDWARE_H
#include "Wippersnapper_V2.h"
#include <Arduino.h>
#include <HardwareSerial.h>

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

/*!
    @brief  Interface for interacting with the UART hardware.
*/
class UARTHardware {
public:
  UARTHardware(const wippersnapper_uart_UartSerialConfig &config);
  ~UARTHardware();
  bool ConfigureSerial();
  uint16_t UartPacketFormatToConfig(
      const wippersnapper_uart_UartPacketFormat uart_format);
  int GetBusNumber();
  bool isHardwareSerial() const;
  bool isSoftwareSerial() const;
  HardwareSerial *GetHardwareSerial();
#if HAS_SW_SERIAL
  SoftwareSerial *GetSoftwareSerial();
#endif // HAS_SW_SERIAL
  uint32_t GetBaudRate();

private:
  wippersnapper_uart_UartSerialConfig
      _config;                         ///< The UART serial configuration
  HardwareSerial *_hwSerial = nullptr; ///< HardwareSerial instance for this bus
#if HAS_SW_SERIAL
  SoftwareSerial *_swSerial = nullptr; ///< SoftwareSerial instance for this bus
#endif                                 // HAS_SW_SERIAL
  int _uart_nbr = -1;  ///< The UART bus number this hardware instance is using
  uint32_t _baud_rate; ///< The baud rate for this hardware instance
};
#endif // WS_UART_HARDWARE_H