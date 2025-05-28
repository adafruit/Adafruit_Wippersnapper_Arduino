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
* @brief  Configures the UART serial using a provided configuration.
* @param  config The configuration for the serial.
* @return True if the serial was successfully configured, False otherwise.
*/
bool UARTHardware::ConfigureSerial(const wippersnapper_uart_UartSerialConfig &config) {
  // TODO: IMPLEMENT HERE
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