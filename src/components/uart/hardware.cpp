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

/*!
    @brief  Constructs a new UARTHardware object.
    @param  config The configuration for the serial.
*/
UARTHardware::UARTHardware(const wippersnapper_uart_UartSerialConfig &config) {
  _config = config;
  _uart_nbr = config.uart_nbr;
}

/*!
    @brief  Destructs the UARTHardware.
*/
UARTHardware::~UARTHardware() {
  // TODO
}

/*!
    @brief  Converts a wippersnapper_uart_UartPacketFormat to a HardwareSerial
   config.
    @param  uart_pkt_fmt
            The UART packet format to convert.
    @return The corresponding xSerial config value.
*/
uint16_t UARTHardware::UartPacketFormatToConfig(
    const wippersnapper_uart_UartPacketFormat uart_pkt_fmt) {
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
 * @return True if the serial was successfully configured, False otherwise.
 */
bool UARTHardware::ConfigureSerial() {
  int8_t rx_pin = -1;
  int8_t tx_pin = -1;
  if (_config.pin_rx[0] == 'D' || _config.pin_rx[0] == 'A') {
    rx_pin = atoi(_config.pin_rx + 1);
  } else {
    rx_pin = atoi(_config.pin_rx);
  }
  if (_config.pin_tx[0] == 'D' || _config.pin_tx[0] == 'A') {
    tx_pin = atoi(_config.pin_tx + 1);
  } else {
    tx_pin = atoi(_config.pin_tx);
  }
  uint16_t cfg = UartPacketFormatToConfig(_config.format);

  if (_config.use_sw_serial) {
// Does this architecture support SoftwareSerial?
#if HAS_SW_SERIAL
    // Create a new SoftwareSerial instance
    _swSerial = new SoftwareSerial(rx_pin, tx_pin, _config.sw_serial_invert);
    if (_swSerial == nullptr) {
      WS_DEBUG_PRINTLN(
          "[uart] ERROR: Failed to allocate a SoftwareSerial instance!");
      return false;
    }
    _swSerial->begin((unsigned long)_config.baud_rate);
    _baud_rate = _config.baud_rate;
#endif // HAS_SW_SERIAL
  } else {
#if ARDUINO_ARCH_ESP32
    // Create a new HardwareSerial instance
    _hwSerial = new HardwareSerial(_config.uart_nbr);
    if (_hwSerial == nullptr)
      return false;
    _hwSerial->begin((unsigned long)_config.baud_rate, (uint32_t)cfg, rx_pin,
                     tx_pin, false, (unsigned long)_config.timeout);
#elif ARDUINO_ARCH_RP2040
    // Create a new SerialUART instance
    uart_inst_t *uart_hw = nullptr;
    // determine which bus to use (RP2040 only supports 2 hardware UARTs)
    if (_config.uart_nbr == 0) {
      uart_hw = uart0;
    } else if (_config.uart_nbr == 1) {
      uart_hw = uart1;
    } else {
      WS_DEBUG_PRINTLN("[uart] ERROR: Invalid UART bus number specified!");
      return false;
    }
    _hwSerial = new SerialUART(uart_hw, tx_pin, rx_pin);
    if (_hwSerial == nullptr) {
      return false;
    }
    _hwSerial->begin((unsigned long)_config.baud_rate, (uint32_t)cfg);
#else
    // ESP8266, SAMD, and other platforms
    // take the default Arduino/Wiring API arguments
    // Create a new HardwareSerial instance
    _hwSerial = new HardwareSerial(_config.uart_nbr);
    if (_hwSerial == nullptr)
      return false;
    _hwSerial->begin((unsigned long)_config.baud_rate, (uint32_t)cfg);
#endif
    _baud_rate = _config.baud_rate;
  }
  return true;
}

/*!
 * @brief  Checks if the hardware instance has allocated and is using a
 * HardwareSerial.
 * @return True if a HardwareSerial instance is in-use by the hardware, False
 * otherwise.
 */
bool UARTHardware::isHardwareSerial() const { return _hwSerial != nullptr; }

/*!
 * @brief  Checks if the hardware instance has allocated and is using a Software
 * Serial.
 * @return True if a SoftwareSerial instance is in-use by the hardware, False
 * otherwise.
 */
bool UARTHardware::isSoftwareSerial() const {
#if HAS_SW_SERIAL
  return _swSerial != nullptr;
#endif
  return false;
}

/*!
 * @brief  Gets the bus number of the hardware instance.
 * @return The bus number of the hardware instance, or -1 if not set.
 */
int UARTHardware::GetBusNumber() { return _uart_nbr; }

/*!
 * @brief  Gets the HardwareSerial instance for this port
 * @return A pointer to the HardwareSerial instance, or nullptr if not
 * constructed.
 */
HardwareSerial *UARTHardware::GetHardwareSerial() { return _hwSerial; }

#if HAS_SW_SERIAL
/*!
 * @brief  Gets the SoftwareSerial instance for this port
 * @return A pointer to the SoftwareSerial instance, or nullptr if not
 * constructed.
 */
SoftwareSerial *UARTHardware::GetSoftwareSerial() { return _swSerial; }
#endif // HAS_SW_SERIAL

/*!
 * @brief  Gets the baud rate of the serial instance.
 * @return The baud rate of the serial instance.
 */
uint32_t UARTHardware::GetBaudRate() { return _baud_rate; }