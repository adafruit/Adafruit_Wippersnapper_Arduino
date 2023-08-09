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

ws_uart::~ws_uart(void) {
#ifdef USE_SW_UART
  _swSerial = nullptr;
#else
  _hwSerial = nullptr;
#endif
}

bool ws_uart::begin(
    wippersnapper_uart_v1_UARTDeviceAttachRequest *msgUARTRequest) {
  // Parse bus_info
  int32_t baud = msgUARTRequest->bus_info.baudrate;
  int32_t rx = msgUARTRequest->bus_info.pin_rx;
  int32_t tx = msgUARTRequest->bus_info.pin_tx;
  bool invert = msgUARTRequest->bus_info.is_invert;

// Initialize and begin UART bus depending on if HW UART or SW UART
#ifdef USE_SW_UART
  // NOTE/TODO: Currently UNTESTED and NOT COMPILED WITH DEFAULT BUILD TARGET,
  // ESP32!
  pinMode(rx, INPUT);
  pinMode(tx, OUTPUT);
  _swSerial = &SoftwareSerial(rx, tx, invert);
#else
  _hwSerial = &HWSerial;
  _hwSerial->begin(baud, SERIAL_8N1, rx, tx, invert);
#endif

  // Initialize UART device
  if (strcmp(msgUARTRequest->device_id, "pm25aqi") == 0) {
    if (_pm25aqi == nullptr) {
      WS_DEBUG_PRINTLN(
          "[ERROR, UART]: PM25AQI driver already initialized on bus!");
      return false;
    }
    // TODO: Add SW serial support below
    _pm25aqi =
        new ws_uart_drv_pm25aqi(_hwSerial, msgUARTRequest->polling_interval);
    if (!_pm25aqi->begin()) {
      WS_DEBUG_PRINTLN("[ERROR, UART]: PM25 driver initialization failed!");
      return false;
    }
    WS_DEBUG_PRINTLN("[INFO, UART]: PM25 UART driver initialized");
  } else if (strcmp(msgUARTRequest->device_id, "gps") == 0) {
    // TODO: GPS UART initialization here
  } else {
    WS_DEBUG_PRINTLN("[ERROR, UART]: Could not find UART device type");
    return false;
  }

  return true;
}
