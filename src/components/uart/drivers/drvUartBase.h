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
#include "../hardware.h"
#include <Arduino.h>
#include <protos/uart.pb.h>

/*!
    @brief  Base class for UART Drivers.
*/
class drvUartBase {

public:
  /*!
      @brief    Instantiates a UART device.
      @param    HardwareSerial
                Pointer to a HardwareSerial instance.
      @param    device_type
                The type of device connected to the UART port.
      @param    driver_name
                The name of the driver.
  */
  drvUartBase(HardwareSerial *hw_serial,
              wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name) {
    _hw_serial = hw_serial;
    _is_software_serial = false;
    _device_type = device_type;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }

#if HAS_SW_SERIAL
  /*!
      @brief    Instantiates a UART device.
      @param    SoftwareSerial
                Pointer to a SoftwareSerial instance.
      @param    device_type
                The type of device connected to the UART port.
      @param    driver_name
                The name of the driver.
  */
  drvUartBase(SoftwareSerial *sw_serial,
              wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name) {
    _sw_serial = sw_serial;
    _is_software_serial = true;
    _device_type = device_type;
    strncpy(_name, driver_name, sizeof(_name) - 1);
    _name[sizeof(_name) - 1] = '\0';
  }
#endif // HAS_SW_SERIAL

  /*!
      @brief    Destructor for a UART device.
  */
  virtual ~drvUartBase() {
    // This is a base class, nothing to clean up!
  }

  /*!
      @brief    Gets the UART device type.
      @returns  The UART device type.
  */
  wippersnapper_uart_UartDeviceType GetDeviceType() { return _device_type; }

  /*!
      @brief    Gets the type of Serial used by this driver.
      @returns  True if this driver uses SoftwareSerial, False if it uses
     HardwareSerial.
  */
  bool IsSoftwareSerial() const { return _is_software_serial; }

  /*!
      @brief    Initializes the UART device.
      @returns  True if initialized successfully, False otherwise.
  */
  virtual bool begin() = 0;

protected:
  HardwareSerial *_hw_serial; ///< Pointer to a HardwareSerial instance
#if HAS_SW_SERIAL
  SoftwareSerial *_sw_serial; ///< Pointer to a SoftwareSerial instance
#endif                        // HAS_SW_SERIAL
  wippersnapper_uart_UartDeviceType _device_type; ///< The UART device type
  char _name[15];                                 ///< The device's name
  bool _is_software_serial =
      false; ///< Indicates if this driver uses SoftwareSerial
};
#endif // DRV_UART_BASE_H