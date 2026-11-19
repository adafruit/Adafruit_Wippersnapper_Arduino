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
    @param  config   The configuration for the serial.
    @param  pin_rx   The RX pin name, e.g. "D16".
    @param  pin_tx   The TX pin name, e.g. "D17".
*/
UARTHardware::UARTHardware(const ws_uart_SerialConfig &config,
                           const char *pin_rx, const char *pin_tx) {
  _config = config;
  // Validate and resolve the pin names once; name + number travel together
  bool rx_ok = _pin_rx.Set(pin_rx);
  bool tx_ok = _pin_tx.Set(pin_tx);
  _pins_valid = rx_ok && tx_ok;
}

/*!
    @brief  Destructs the UARTHardware.
*/
UARTHardware::~UARTHardware() {
  if (_hwSerial != nullptr) {
    _hwSerial->end();
    // Only delete if we heap-allocated it; on platforms where we
    // assigned &Serial1/&Serial2 the pointer is a global reference.
#if !defined(ARDUINO_ARCH_RP2040) && !defined(ADAFRUIT_METRO_M4_EXPRESS) &&    \
    !defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) && !defined(ADAFRUIT_PYPORTAL) && \
    !defined(ADAFRUIT_PYPORTAL_M4_TITANO) && !defined(ARDUINO_ARCH_SAMD)
    delete _hwSerial;
#endif
    _hwSerial = nullptr;
  }
#if HAS_SW_SERIAL
  if (_swSerial != nullptr) {
    _swSerial->end();
    delete _swSerial;
    _swSerial = nullptr;
  }
#endif
}

/*!
    @brief  Converts a ws_uart_PacketFormat to a HardwareSerial
   config.
    @param  uart_pkt_fmt
            The UART packet format to convert.
    @return The corresponding xSerial config value.
*/
uint16_t UARTHardware::UartPacketFormatToConfig(
    const ws_uart_PacketFormat uart_pkt_fmt) {
  switch (uart_pkt_fmt) {
  case ws_uart_PacketFormat_PF_8N1:
    return SERIAL_8N1;
  case ws_uart_PacketFormat_PF_5N1:
    return SERIAL_5N1;
  case ws_uart_PacketFormat_PF_6N1:
    return SERIAL_6N1;
  case ws_uart_PacketFormat_PF_7N1:
    return SERIAL_7N1;
  case ws_uart_PacketFormat_PF_5N2:
    return SERIAL_5N2;
  case ws_uart_PacketFormat_PF_6N2:
    return SERIAL_6N2;
  case ws_uart_PacketFormat_PF_7N2:
    return SERIAL_7N2;
  case ws_uart_PacketFormat_PF_8N2:
    return SERIAL_8N2;
  case ws_uart_PacketFormat_PF_5E1:
    return SERIAL_5E1;
  case ws_uart_PacketFormat_PF_6E1:
    return SERIAL_6E1;
  case ws_uart_PacketFormat_PF_7E1:
    return SERIAL_7E1;
  case ws_uart_PacketFormat_PF_8E1:
    return SERIAL_8E1;
  case ws_uart_PacketFormat_PF_5E2:
    return SERIAL_5E2;
  case ws_uart_PacketFormat_PF_6E2:
    return SERIAL_6E2;
  case ws_uart_PacketFormat_PF_7E2:
    return SERIAL_7E2;
  case ws_uart_PacketFormat_PF_8E2:
    return SERIAL_8E2;
  case ws_uart_PacketFormat_PF_5O1:
    return SERIAL_5O1;
  case ws_uart_PacketFormat_PF_6O1:
    return SERIAL_6O1;
  case ws_uart_PacketFormat_PF_7O1:
    return SERIAL_7O1;
  case ws_uart_PacketFormat_PF_8O1:
    return SERIAL_8O1;
  case ws_uart_PacketFormat_PF_5O2:
    return SERIAL_5O2;
  case ws_uart_PacketFormat_PF_6O2:
    return SERIAL_6O2;
  case ws_uart_PacketFormat_PF_7O2:
    return SERIAL_7O2;
  case ws_uart_PacketFormat_PF_8O2:
    return SERIAL_8O2;
  default:
    return SERIAL_8N1;
  }
}

#if defined(ARDUINO_ARCH_ESP8266) && HAS_SW_SERIAL
/*!
    @brief  Converts a ws_uart_PacketFormat to an ESP8266 SoftwareSerial
            (EspSoftwareSerial) config, whose values differ from
            HardwareSerial's.
    @param  uart_pkt_fmt
            The UART packet format to convert.
    @return The corresponding EspSoftwareSerial::Config value.
*/
static EspSoftwareSerial::Config
UartPacketFormatToSwConfig(const ws_uart_PacketFormat uart_pkt_fmt) {
  switch (uart_pkt_fmt) {
  case ws_uart_PacketFormat_PF_5N1:
    return EspSoftwareSerial::SWSERIAL_5N1;
  case ws_uart_PacketFormat_PF_6N1:
    return EspSoftwareSerial::SWSERIAL_6N1;
  case ws_uart_PacketFormat_PF_7N1:
    return EspSoftwareSerial::SWSERIAL_7N1;
  case ws_uart_PacketFormat_PF_5N2:
    return EspSoftwareSerial::SWSERIAL_5N2;
  case ws_uart_PacketFormat_PF_6N2:
    return EspSoftwareSerial::SWSERIAL_6N2;
  case ws_uart_PacketFormat_PF_7N2:
    return EspSoftwareSerial::SWSERIAL_7N2;
  case ws_uart_PacketFormat_PF_8N2:
    return EspSoftwareSerial::SWSERIAL_8N2;
  case ws_uart_PacketFormat_PF_5E1:
    return EspSoftwareSerial::SWSERIAL_5E1;
  case ws_uart_PacketFormat_PF_6E1:
    return EspSoftwareSerial::SWSERIAL_6E1;
  case ws_uart_PacketFormat_PF_7E1:
    return EspSoftwareSerial::SWSERIAL_7E1;
  case ws_uart_PacketFormat_PF_8E1:
    return EspSoftwareSerial::SWSERIAL_8E1;
  case ws_uart_PacketFormat_PF_5E2:
    return EspSoftwareSerial::SWSERIAL_5E2;
  case ws_uart_PacketFormat_PF_6E2:
    return EspSoftwareSerial::SWSERIAL_6E2;
  case ws_uart_PacketFormat_PF_7E2:
    return EspSoftwareSerial::SWSERIAL_7E2;
  case ws_uart_PacketFormat_PF_8E2:
    return EspSoftwareSerial::SWSERIAL_8E2;
  case ws_uart_PacketFormat_PF_5O1:
    return EspSoftwareSerial::SWSERIAL_5O1;
  case ws_uart_PacketFormat_PF_6O1:
    return EspSoftwareSerial::SWSERIAL_6O1;
  case ws_uart_PacketFormat_PF_7O1:
    return EspSoftwareSerial::SWSERIAL_7O1;
  case ws_uart_PacketFormat_PF_8O1:
    return EspSoftwareSerial::SWSERIAL_8O1;
  case ws_uart_PacketFormat_PF_5O2:
    return EspSoftwareSerial::SWSERIAL_5O2;
  case ws_uart_PacketFormat_PF_6O2:
    return EspSoftwareSerial::SWSERIAL_6O2;
  case ws_uart_PacketFormat_PF_7O2:
    return EspSoftwareSerial::SWSERIAL_7O2;
  case ws_uart_PacketFormat_PF_8O2:
    return EspSoftwareSerial::SWSERIAL_8O2;
  default: // PF_8N1 and PF_UNSPECIFIED
    return EspSoftwareSerial::SWSERIAL_8N1;
  }
}
#endif // ARDUINO_ARCH_ESP8266 && HAS_SW_SERIAL

/*!
 * @brief  Configures the UART serial using a provided configuration.
 * @return True if the serial was successfully configured, False otherwise.
 */
bool UARTHardware::ConfigureSerial() {
  if (!_pins_valid) {
    WS_DEBUG_PRINTLN("[uart] ERROR: Invalid RX/TX pin name!");
    return false;
  }
  int8_t rx_pin = (int8_t)_pin_rx.num;
  int8_t tx_pin = (int8_t)_pin_tx.num;
  uint16_t cfg = UartPacketFormatToConfig(_config.format);

  // SerialConfig no longer carries use_sw_serial (adafruit/
  // Wippersnapper_Protobuf#230), so the device picks. Board definitions
  // currently expose a single UART on its default pins. ESP8266 needs
  // SoftwareSerial there (UART0 is the USB/debug Serial and UART1 is TX-only);
  // other architectures use that default hardware UART.
#if defined(ARDUINO_ARCH_ESP8266) && HAS_SW_SERIAL
  bool use_sw_serial = true;
#else
  bool use_sw_serial = false;
#endif
  if (use_sw_serial) {
// Does this architecture support SoftwareSerial?
#if HAS_SW_SERIAL
    // Create a new SoftwareSerial instance
    _swSerial = new SoftwareSerial(rx_pin, tx_pin, _config.sw_serial_invert);
    if (_swSerial == nullptr) {
      WS_DEBUG_PRINTLN(
          "[uart] ERROR: Failed to allocate a SoftwareSerial instance!");
      return false;
    }
#if defined(ARDUINO_ARCH_ESP8266)
    _swSerial->begin((uint32_t)_config.baud_rate,
                     UartPacketFormatToSwConfig(_config.format));
#else
    _swSerial->begin((unsigned long)_config.baud_rate);
#endif
    _baud_rate = _config.baud_rate;
#endif // HAS_SW_SERIAL
  } else {
#if defined(ARDUINO_ARCH_RP2040) || defined(ADAFRUIT_METRO_M4_EXPRESS) ||      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO) || defined(ARDUINO_ARCH_SAMD)
    _hwSerial = &Serial1;
#else
    // Create a new HardwareSerial instance (use UART1 by default)
    _hwSerial = new HardwareSerial(1);
#endif
    if (_hwSerial == nullptr) {
      WS_DEBUG_PRINTLN(
          "[uart] ERROR: Failed to allocate HardwareSerial instance!");
      return false;
    }
// Calls to HardwareSerial begin() differ based on the platform
#if ARDUINO_ARCH_ESP32
    _hwSerial->begin((unsigned long)_config.baud_rate, (uint32_t)cfg, rx_pin,
                     tx_pin, false, (unsigned long)_config.timeout);
#elif ARDUINO_ARCH_RP2040
    // Serial1 runs on its default pins, which is all board definitions expose.
    // TODO: to support other pins, use Serial1/Serial2 (UART0/UART1) when the
    // RX/TX pair belongs to that UART and it is free, calling setRX/setTX
    // first. Check the pair against arduino-pico's SerialUART::setRX/setTX pin
    // lists beforehand: they panic on an invalid pin. Otherwise fall back to
    // SoftwareSerial, which arduino-pico implements with PIO (SerialPIO): check
    // `if (!*_swSerial)` after begin() for "no free PIO state machine", since
    // Servo and the Pico W radio share the PIO state machines.
    _hwSerial->begin((unsigned long)_config.baud_rate, (uint32_t)cfg);
#else
// ESP8266, SAMD, and other platforms
// take the default Arduino/Wiring API arguments
#ifdef ARDUINO_ARCH_ESP8266
    // We want to pass the cfg as an enum for ESP8266
    _hwSerial->begin((unsigned long)_config.baud_rate, (SerialConfig)cfg);
#else
    // We want to cast the cfg for other platforms
    _hwSerial->begin((unsigned long)_config.baud_rate, (uint32_t)cfg);
#endif
#endif
    _baud_rate = _config.baud_rate;
  }

  // SerialConfig.timeout is the maximum time to wait for serial data, in ms
  // (proto default 1000 ms). Apply it as the Stream read timeout used by
  // readBytes()/readString() etc. on every architecture. (ESP32's begin()
  // timeout argument above is only the auto-baud detection timeout.)
  unsigned long timeout_ms =
      _config.timeout > 0 ? (unsigned long)_config.timeout : ONE_SECOND_IN_MS;
  if (_hwSerial != nullptr)
    _hwSerial->setTimeout(timeout_ms);
#if HAS_SW_SERIAL
  if (_swSerial != nullptr)
    _swSerial->setTimeout(timeout_ms);
#endif // HAS_SW_SERIAL
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
int UARTHardware::getPortNum() { return (int)_pin_rx.num; }

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