/*!
 * @file drvUartPm25.h
 *
 * Interface for the Adafruit_PM25AQI UART driver.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_UART_PM25_H
#define DRV_UART_PM25_H
#include "Adafruit_PM25AQI.h"
#include "drvUartBase.h"

/**************************************************************************/
/*!
    @brief  Provides an interface for the Adafruit_PM25AQI library over
            UART.
*/
/**************************************************************************/
class drvUartPm25 : public drvUartBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Instantiates a UART device.
      @param    device_type
                The type of device connected to the UART port.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvUartPm25(wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name)
      : drvUartBase(device_type, driver_name) {
    _device_type = device_type;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a UART device.
  */
  /*******************************************************************************/
  ~drvUartPm25() {
    // This is a base class, nothing to clean up!
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the UART device.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    // TODO!
    return true;
  }

protected:
  Adafruit_PM25AQI *_aqi = nullptr; ///< Instance of the Adafruit_PM25AQI class
};
#endif // DRV_UART_PM25_H