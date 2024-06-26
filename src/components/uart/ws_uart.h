/*!
 * @file ws_uart.h
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
#ifndef WS_UART_H
#define WS_UART_H

#include "Wippersnapper.h"
#include "drivers/ws_uart_drv.h"
#include "drivers/ws_uart_drv_pm25aqi.h"

/**************************************************************************/
/*!
    @brief  Class that provides an interface between WipperSnapper's app
            and the device's UART bus.
*/
/**************************************************************************/
class ws_uart {
public:
  ws_uart(){};
  ~ws_uart(void);

  void
  initUARTBus(wippersnapper_uart_v1_UARTDeviceAttachRequest
                  *msgUARTRequest); ///< Initializes the UART bus, called once
  bool initUARTDevice(wippersnapper_uart_v1_UARTDeviceAttachRequest
                          *msgUARTRequest); ///< Initializes a UART driver.
  bool isUARTBusInitialized(); ///< Returns true if the UART bus is initialized
  void detachUARTDevice(
      wippersnapper_uart_v1_UARTDeviceDetachRequest
          *msgUARTDetachReq); ///< Detaches a UART device from the UART bus
  void deinitUARTDevice(const char *device_id);
  void update(); ///< Updates the UART device at every polling interval, must be
                 ///< called by main app.
#ifdef USE_SW_UART
  bool initUARTDevicePM25AQI(SoftwareSerial *swSerial, int32_t pollingInterval,
                             const char *device_id);
#else
  bool initUARTDevicePM25AQI(HardwareSerial *hwSerial, int32_t pollingInterval,
                             const char *device_id);
#endif
private:
#ifdef USE_SW_UART
  SoftwareSerial *_swSerial = nullptr; ///< SoftwareSerial instance
#else
  HardwareSerial *_hwSerial = nullptr; ///< HardwareSerial instance
#endif
  bool _is_bus_initialized = false;        ///< True if UART bus is initialized
  std::vector<ws_uart_drv *> uartDrivers;  ///< Vector of UART drivers
  ws_uart_drv_pm25aqi *_pm25aqi = nullptr; ///< Pointer to a PM25 AQI device
};

#endif // WS_UART_H