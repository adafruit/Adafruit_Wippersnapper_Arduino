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

/*!
    @brief  Provides an interface for the Adafruit_PM25AQI library over
            UART.
*/
class drvUartPm25 : public drvUartBase {

public:
  /*!
      @brief    Instantiates a UART device.
      @param    hw_serial
                Pointer to a HardwareSerial instance.
      @param    device_type
                The type of device connected to the UART port.
      @param    driver_name
                The name of the driver.
  */
  drvUartPm25(HardwareSerial *hw_serial,
              wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name)
      : drvUartBase(hw_serial, device_type, driver_name) {
    // Handled by drvUartBase constructor
  }

#if HAS_SW_SERIAL
  /*!
    @brief    Instantiates a UART device.
    @param    sw_serial
              Pointer to a SoftwareSerial instance.
    @param    device_type
              The type of device connected to the UART port.
    @param    driver_name
              The name of the driver.
*/
  drvUartPm25(SoftwareSerial *sw_serial,
              wippersnapper_uart_UartDeviceType device_type,
              const char *driver_name)
      : drvUartBase(sw_serial, device_type, driver_name) {
    // Handled by drvUartBase constructor
  }
#endif // HAS_SW_SERIAL

  /*!
      @brief    Destructor for a UART device.
  */
  ~drvUartPm25() {
    if (_aqi) {
      delete _aqi; // Clean up the Adafruit_PM25AQI instance
      _aqi = nullptr;
    }
  }

  /*!
      @brief    Initializes the Adafruit_PM25AQI instance.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    if (IsSoftwareSerial)
      return _aqi->begin(_hw_serial);
    return _aqi->begin(_sw_serial);
  }

protected:
  Adafruit_PM25AQI *_aqi = nullptr; ///< Instance of the Adafruit_PM25AQI class
};
#endif // DRV_UART_PM25_H