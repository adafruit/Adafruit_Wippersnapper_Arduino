/*!
 * @file ws_uart_driver_pm25aqi.h
 *
 * Device driver for the Adafruit PM25AQI Arduino Library
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_UART_DRV_PM25AQI_H
#define WS_UART_DRV_PM25AQI_H

#include "ws_uart_drv.h"
#include <Adafruit_PM25AQI.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a
            UART PM25 AQI sensor.
*/
/**************************************************************************/
class ws_uart_drv_pm25aqi : public ws_uart_drv {
public:
  // TODO
  ws_uart_drv_pm25aqi() : ws_uart_drv() {}

  ~ws_uart_drv_pm25aqi() {
    _aqi = nullptr;
  }


protected:
   Adafruit_PM25AQI *_aqi = nullptr; ///< Pointer to PM25AQI sensor object
};

#endif // WipperSnapper_I2C_Driver_VL53L0X