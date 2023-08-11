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

/*******************************************************************************/
/*!
    @brief    UART class destructor.
*/
/*******************************************************************************/
ws_uart::~ws_uart(void) {
#ifdef USE_SW_UART
  _swSerial = nullptr;
#else
  _hwSerial = nullptr;
#endif
}

// TODO: Maybe we need two begin functions, one for the bus, one for the device?
/*******************************************************************************/
/*!
    @brief    Initializes a UART bus and the device on the bus.
    @param    msgUARTRequest
              Pointer to a UARTDeviceAttachRequest message.
    @returns  True if UART bus initialized successfully, False otherwise.
*/
/*******************************************************************************/
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
    // Set MQTT client in driver TODO: This should be handled better, elsewhere!
    _pm25aqi->set_mqtt_client(WS._mqtt);
  } else if (strcmp(msgUARTRequest->device_id, "gps") == 0) {
    // TODO: GPS UART initialization here
  } else {
    WS_DEBUG_PRINTLN("[ERROR, UART]: Could not find UART device type");
    return false;
  }
  return true;
}

/*******************************************************************************/
/*!
    @brief    Checks each UART driver's polling interval and sends an update of
              the device's state to IO.
*/
/*******************************************************************************/
void ws_uart::update() {
  if (_pm25aqi == nullptr) {
    return; // No driver initialized on bus to update
  }

  long curTime = millis();
  // Check if PM25AQI driver is ready to poll
  if (curTime - _pm25aqi->lastPoll >= _pm25aqi->pollingInterval) {
    if (!_pm25aqi->data_available()) {
      WS_DEBUG_PRINTLN("[ERROR, UART]: PM25AQI data not ready yet!");
      return;
    }
    // Update IO with reading
    _pm25aqi->update();
    // Set lastPoll time
    _pm25aqi->lastPoll = curTime;
  }
}
