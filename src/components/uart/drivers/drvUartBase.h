/*!
 * @file drvUartBase.h
 *
 * Base implementation for UART device drivers.
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

#ifndef DRV_UART_BASE_H
#define DRV_UART_BASE_H
#include <Arduino.h>
#include <protos/uart.pb.h>

/**************************************************************************/
/*!
    @brief  Base class for UART Drivers.
*/
/**************************************************************************/
class drvUartBase {

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
  drvUartBase(wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name) {
    // TODO: This needs a reference to the UART hardware instance!!!
    _device_type = device_type;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a UART device.
  */
  /*******************************************************************************/
  virtual ~drvUartBase() {
    // This is a base class, nothing to clean up!
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the UART device type.
      @returns  The UART device type.
  */
  /*******************************************************************************/
  wippersnapper_uart_UartDeviceType GetDeviceType() { return _device_type; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the UART device.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  virtual bool begin() = 0;

protected:
  wippersnapper_uart_UartDeviceType _device_type; ///< The UART device type
  char _name[15];                                 ///< The device's name
};
#endif // DRV_UART_BASE_H