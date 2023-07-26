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

ws_uart:ws_uart(wippersnapper_uart_v1_UARTDeviceAttachRequest *msgUARTRequest) {
  // Parse out message's bus_info
  int32_t baud = msgUARTRequest->bus_info.baudrate;


}

ws_uart::~ws_uart(void) {
    _swSerial = nullptr;
    _hwSerial = nullptr;
}