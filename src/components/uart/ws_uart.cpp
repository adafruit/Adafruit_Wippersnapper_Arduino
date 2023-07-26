/*!
 * @file ws_uart.cpp
 *
 * Base class that provides an interface between WipperSnapper's app
 * and the device's UART bus.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "ws_uart.h"

#if !defined(ARDUINO_ARCH_ESP8266) || !defined(ARDUINO_ARCH_RP2040)
HardwareSerial HWSerial(1); ///< Default HardwareSerial instance
#endif

ws_uart::ws_uart(
    wippersnapper_uart_v1_UARTDeviceAttachRequest *msgUARTRequest) {
  // Parse out message's bus_info and store in class
  int32_t baud = msgUARTRequest->bus_info.baudrate;
  int32_t rx = msgUARTRequest->bus_info.pin_rx;
  int32_t tx = msgUARTRequest->bus_info.pin_tx;
  bool invert = msgUARTRequest->bus_info.is_invert;

  // Initialize and begin UART hardware serial bus
  _hwSerial = &HWSerial;
  _hwSerial->begin(baud, SERIAL_8N1, rx, tx, invert);
  // TODO: Create UART software serial bus

  // TODO: Handle parsing out the device's info too
}

ws_uart::~ws_uart(void) {
#ifdef USE_SW_UART
  _swSerial = nullptr;
#else
  _hwSerial = nullptr;
#endif
}
