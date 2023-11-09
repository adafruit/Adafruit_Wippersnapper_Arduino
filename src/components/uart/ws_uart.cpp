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

// setPins() will detach any previous pins that have been changed.
#ifdef USE_SW_UART
  _swSerial->setPins();
#else
#ifndef ARDUINO_ARCH_SAMD
  _hwSerial->setPins();
#endif
}

/*******************************************************************************/
/*!
    @brief    Initializes a UART bus.
    @param    msgUARTRequest
              Pointer to a UARTDeviceAttachRequest message.
*/
/*******************************************************************************/
void ws_uart::initUARTBus(
    wippersnapper_uart_v1_UARTDeviceAttachRequest *msgUARTRequest) {
  // Parse bus_info
  int32_t baud = msgUARTRequest->bus_info.baudrate;
  int32_t rx = atoi(msgUARTRequest->bus_info.pin_rx + 1);
  int32_t tx = atoi(msgUARTRequest->bus_info.pin_tx + 1);
  bool invert = msgUARTRequest->bus_info.is_invert;

// Initialize and begin UART bus depending if the platform supports either HW
// UART or SW UART
#ifdef USE_SW_UART
#ifndef ARDUINO_ARCH_RP2040
  _swSerial = new SoftwareSerial(rx, tx, invert);
#else // RP2040 SoftwareSerial emulation does not support inverted mode
  _swSerial = new SoftwareSerial(rx, tx);
#endif
  _swSerial->begin(baud);
#else
#ifndef ARDUINO_ARCH_SAMD
  _hwSerial = new HardwareSerial(1);
  _hwSerial->begin(baud, SERIAL_8N1, rx, tx, invert);
#else
  _hwSerial = &Serial1;
  _hwSerial->begin(baud);
#endif
#endif
  _is_bus_initialized = true;
}

#ifdef USE_SW_UART
/*******************************************************************************/
/*!
    @brief    Initializes the pms5003 device driver using SoftwareSerial.
    @param    swSerial
              Pointer to a SoftwareSerial instance.
    @param    pollingInterval
              Polling interval for the pms5003 device.
    @returns  True if pms5003 driver was successfully initialized, False
   otherwise.
*/
/*******************************************************************************/
bool ws_uart::initUARTDevicePM25AQI(SoftwareSerial *swSerial,
                                    int32_t pollingInterval) {
  if (_pm25aqi != nullptr) {
    WS_DEBUG_PRINTLN(
        "[ERROR, UART]: pms5003 driver already initialized on bus!");
    return false;
  }
  WS_DEBUG_PRINTLN("[INFO, UART]: Initializing PM25AQI driver...");
  _pm25aqi = new ws_uart_drv_pm25aqi(swSerial, pollingInterval);
  if (!_pm25aqi->begin()) {
    WS_DEBUG_PRINTLN("[ERROR, UART]: PM25 driver initialization failed!");
    return false;
  }
  _pm25aqi->set_mqtt_client(WS._mqtt, WS._topic_signal_uart_device);
  uartDrivers.push_back(_pm25aqi);
  return true;
}
#else
/*******************************************************************************/
/*!
    @brief    Initializes the pms5003 device driver using HardwareSerial.
    @param    hwSerial
              Pointer to a HardwareSerial instance.
    @param    pollingInterval
              Polling interval for the pms5003 device.
    @returns  True if pms5003 driver was successfully initialized, False
   otherwise.
*/
/*******************************************************************************/
bool ws_uart::initUARTDevicePM25AQI(HardwareSerial *hwSerial,
                                    int32_t pollingInterval) {
  if (_pm25aqi != nullptr) {
    WS_DEBUG_PRINTLN(
        "[ERROR, UART]: pms5003 driver already initialized on bus!");
    return false;
  }
  _pm25aqi = new ws_uart_drv_pm25aqi(hwSerial, pollingInterval);
  if (!_pm25aqi->begin()) {
    WS_DEBUG_PRINTLN("[ERROR, UART]: PM25 driver initialization failed!");
    return false;
  }
  _pm25aqi->set_mqtt_client(WS._mqtt, WS._topic_signal_uart_device);
  uartDrivers.push_back(_pm25aqi);
  return true;
}
#endif

/*******************************************************************************/
/*!
    @brief    Checks if the UART bus has been initialized.
    @returns  True if the UART bus is initialized, False otherwise.
*/
/*******************************************************************************/
bool ws_uart::isUARTBusInitialized() { return _is_bus_initialized; }

/*******************************************************************************/
/*!
    @brief    Initializes a device on the UART bus.
    @param    msgUARTRequest
              Pointer to a UARTDeviceAttachRequest message.
    @returns  True if UART driver was successfully initialized.
*/
/*******************************************************************************/
bool ws_uart::initUARTDevice(
    wippersnapper_uart_v1_UARTDeviceAttachRequest *msgUARTRequest) {
  // Ensure the protobuf contains a device identifier
  if (strlen(msgUARTRequest->device_id) == 0) {
    return false;
  }

  // Do we already have a device with this ID?
  for (ws_uart_drv *ptrUARTDriver : uartDrivers) {
    if (strcmp(ptrUARTDriver->getDeviceID(), msgUARTRequest->device_id) == 0) {
      deinitUARTDevice(
          msgUARTRequest->device_id); // Deinit the device and free resources
    }
  }

  // Check which device type we are initializing
  if (strcmp(msgUARTRequest->device_id, "pms5003") == 0) {
// Attempt to initialize PMS5003 driver with either SW or HW UART
#ifdef USE_SW_UART
    if (!initUARTDevicePM25AQI(_swSerial, msgUARTRequest->polling_interval))
      return false;
#else
    if (!initUARTDevicePM25AQI(_hwSerial, msgUARTRequest->polling_interval))
      return false;
#endif
    WS_DEBUG_PRINTLN("[INFO, UART]: PM25 UART driver initialized!");
  } else {
    WS_DEBUG_PRINTLN("[ERROR, UART]: Could not find UART device type");
    return false;
  }
  return true;
}

/*******************************************************************************/
/*!
    @brief    Deinitializes a device from the UART bus and frees its memory.
    @param    device_id
              Device identifier of the UART device to deinitialize.
*/
/*******************************************************************************/
void ws_uart::deinitUARTDevice(const char *device_id) {
  // Start an iterator on the first driver within the uartDrivers vector
  std::vector<ws_uart_drv *>::iterator iter = uartDrivers.begin();
  // Iterate through the vector
  while (iter != uartDrivers.end()) {
    ws_uart_drv *ptrUARTDriver = *iter; // Get a pointer to the driver
    if (strcmp(ptrUARTDriver->getDeviceID(), device_id) == 0) {
      if (ptrUARTDriver == _pm25aqi) {
        _pm25aqi = nullptr;
      }
      // Erase the driver from the vector of drivers
      iter = uartDrivers.erase(iter);
    } else {
      ++iter;
    }
  }
}

/*******************************************************************************/
/*!
    @brief    Detaches a UART device from the bus and frees its memory.
    @param    msgUARTDetachReq
              Pointer to a UARTDeviceDetachRequest message.
*/
/*******************************************************************************/
void ws_uart::detachUARTDevice(
    wippersnapper_uart_v1_UARTDeviceDetachRequest *msgUARTDetachReq) {
  // Deallocate the memory pointed to by the driver and detach it from the bus
  deinitUARTDevice(msgUARTDetachReq->device_id);
}

/*******************************************************************************/
/*!
    @brief    Polls the UART driver for new data and sends it to IO.
*/
/*******************************************************************************/
void ws_uart::update() {
  for (ws_uart_drv *ptrUARTDriver : uartDrivers) {
    if (ptrUARTDriver->isReady()) {
      // Attempt to poll the UART driver for new data
      if (ptrUARTDriver->read_data()) {
        // Send UART driver's data to IO
        ptrUARTDriver->send_data();
      }
    }
  }
}
