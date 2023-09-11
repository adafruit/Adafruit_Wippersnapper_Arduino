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

// Initialize and begin UART bus depending on if HW UART or SW UART
#ifdef USE_SW_UART
  // NOTE/TODO: Currently UNTESTED and NOT COMPILED WITH DEFAULT BUILD TARGET,
  // ESP32!
  pinMode(rx, INPUT);
  pinMode(tx, OUTPUT);
  _swSerial = &SoftwareSerial(rx, tx, invert);
#else
  _hwSerial = &HWSerial;
  WS_DEBUG_PRINTLN("[INFO, UART]: Initializing HW UART bus...");
  WS_DEBUG_PRINTLN("[INFO, UART]: Baud Rate: " + String(baud));
  WS_DEBUG_PRINTLN("[INFO, UART]: RX Pin: " + String(rx));
  WS_DEBUG_PRINTLN("[INFO, UART]: TX Pin: " + String(tx));

  _hwSerial->begin(baud, SERIAL_8N1, rx, tx, invert);
  WS_DEBUG_PRINT("UART Topic: " );
  WS_DEBUG_PRINTLN(WS._topic_signal_uart_device);
#endif
  is_bus_initialized = true;
}

/*******************************************************************************/
/*!
    @brief    Initializes the pms5003 device driver.
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
  // Set MQTT client in driver TODO: This should be handled better, elsewhere!?
  _pm25aqi->set_mqtt_client(WS._mqtt);
  uartDrivers.push_back(_pm25aqi);

  WS_DEBUG_PRINT("[INFO, UART]: Polling Interval: ");
  WS_DEBUG_PRINTLN(pollingInterval);
  return true;
}

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
  if (strcmp(msgUARTRequest->device_id, "pms5003") == 0) {
    // Attempt to initialize pms5003 driver
    if (!initUARTDevicePM25AQI(_hwSerial, msgUARTRequest->polling_interval))
      return false;
    WS_DEBUG_PRINTLN("[INFO, UART]: PM25 UART driver initialized!");
  } else {
    WS_DEBUG_PRINTLN("[ERROR, UART]: Could not find UART device type");
    return false;
  }
  return true;
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
  // Start an iterator on the first driver within the uartDrivers vector
  std::vector<ws_uart_drv *>::iterator iter = uartDrivers.begin();
  // Iterate through the vector
  while (iter != uartDrivers.end()) {
    ws_uart_drv *ptrUARTDriver = *iter; // Get a pointer to the driver
    if (strcmp(ptrUARTDriver->getDeviceID(), msgUARTDetachReq->device_id) ==
        0) {
      // Deallocate the memory pointed to by the driver
      delete ptrUARTDriver;
      // Erase the driver from the vector of drivers
      iter = uartDrivers.erase(iter);
    } else {
      ++iter;
    }
  }
}

/*******************************************************************************/
/*!
    @brief    Checks each UART driver's polling interval and sends an update
   of the device's state to IO.
*/
/*******************************************************************************/
void ws_uart::update() {
  for (ws_uart_drv *ptrUARTDriver : uartDrivers) {
    // Is the UART driver ready to be polled?
    if (ptrUARTDriver->isReady()) {
      //WS_DEBUG_PRINTLN("[INFO, UART]: UART driver is ready to be polled.");
      // Does our driver contain new data?
      if (ptrUARTDriver->data_available()) {
        WS_DEBUG_PRINTLN("[INFO, UART]: UART driver has new data.");
        // Update driver's data and send to Adafruit IO
        ptrUARTDriver->update();
        ptrUARTDriver->setPrvPollTime(millis());
      }
    }
  }
}
