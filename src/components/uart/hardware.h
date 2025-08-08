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
#include "serial_config.h"
#include <Arduino.h>
#include <HardwareSerial.h>
#ifdef ARDUINO_ARCH_RP2040
#include <SerialUART.h>
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
      _config; ///< The UART serial configuration
#ifdef ARDUINO_ARCH_RP2040
  SerialUART *_hwSerial = nullptr; ///< Pointer to the SerialUART instance
#else
  HardwareSerial *_hwSerial = nullptr; ///< HardwareSerial instance for this bus
#endif
#if HAS_SW_SERIAL
  SoftwareSerial *_swSerial = nullptr; ///< SoftwareSerial instance for this bus
#endif                                 // HAS_SW_SERIAL
  int _uart_nbr = -1;  ///< The UART bus number this hardware instance is using
  uint32_t _baud_rate; ///< The baud rate for this hardware instance
};
#endif // WS_UART_HARDWARE_H