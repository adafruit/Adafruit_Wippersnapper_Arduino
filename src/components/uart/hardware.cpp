/*!
 * @file src/components/uart/hardware.cpp
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
#include "hardware.h"

/**************************************************************************/
/*!
    @brief  Constructs a new UARTHardware object.
*/
/**************************************************************************/
UARTHardware::UARTHardware() {
  // TODO
}

/**************************************************************************/
/*!
    @brief  Destructs the UARTHardware.
*/
/**************************************************************************/
UARTHardware::~UARTHardware() {
  // TODO
}

/*!
    @brief  Converts a wippersnapper_uart_UartPacketFormat to a HardwareSerial config.
    @param  uart_pkt_fmt
            The UART packet format to convert.
    @return The corresponding xSerial config value.
*/
uint8_t UARTHardware::UartPacketFormatToConfig(const wippersnapper_uart_UartPacketFormat uart_pkt_fmt) {
  switch (uart_pkt_fmt) {
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_8N1:
      return SERIAL_8N1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_5N1:
      return SERIAL_5N1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_6N1:
      return SERIAL_6N1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_7N1:
      return SERIAL_7N1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_5N2:
      return SERIAL_5N2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_6N2:
      return SERIAL_6N2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_7N2:
      return SERIAL_7N2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_8N2:
      return SERIAL_8N2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_5E1:
      return SERIAL_5E1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_6E1:
      return SERIAL_6E1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_7E1:
      return SERIAL_7E1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_8E1:
      return SERIAL_8E1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_5E2:
      return SERIAL_5E2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_6E2:
      return SERIAL_6E2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_7E2:
      return SERIAL_7E2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_8E2:
      return SERIAL_8E2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_5O1:
      return SERIAL_5O1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_6O1:
      return SERIAL_6O1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_7O1:
      return SERIAL_7O1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_8O1:
      return SERIAL_8O1;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_5O2:
      return SERIAL_5O2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_6O2:
      return SERIAL_6O2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_7O2:
      return SERIAL_7O2;
    case wippersnapper_uart_UartPacketFormat_UART_PACKET_FORMAT_8O2:
      return SERIAL_8O2;
    default:
      return SERIAL_8N1;
  }
}

/*!
* @brief  Configures the UART serial using a provided configuration.
* @param  config The configuration for the serial.
* @return True if the serial was successfully configured, False otherwise.
*/
bool UARTHardware::ConfigureSerial(const wippersnapper_uart_UartSerialConfig &config) {
  // TODO: IMPLEMENT HERE
  uint8_t config = UartPacketFormatToConfig(config.format);
  return true;
}

/*!
* @brief  Checks if the hardware instance has allocated and is using a HardwareSerial.
* @return True if a HardwareSerial instance is in-use by the hardware, False otherwise.
*/
bool UARTHardware::isHardwareSerial() const {
  return _hwSerial != nullptr;
}

/*!
* @brief  Checks if the hardware instance has allocated and is using a Software Serial.
* @return True if a SoftwareSerial instance is in-use by the hardware, False otherwise.
*/
bool UARTHardware::isSoftwareSerial() const {
  #if HAS_SW_SERIAL
  return _swSerial != nullptr;
  #endif
  return false;
}