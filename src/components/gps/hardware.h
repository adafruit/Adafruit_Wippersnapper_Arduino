/*!
 * @file src/components/gps/hardware.h
 *
 * Low-level hardware implementation for WipperSnapper's GPS component.
 * Supports both UART and I2C interfaces for GPS devices.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_GPS_HARDWARE_H
#define WS_GPS_HARDWARE_H
#include "Wippersnapper_V2.h"
#include <Arduino.h>

/*!
    @brief  Interface for interacting with the GPS hardware.
            Supports both UART and I2C interfaces.
*/
class GPSHardware {
public:
  // TODO: How are we going to handle the WS_UART_HARDWARE and WS_I2C_HARDWARE interfaces and pass them in?
  // TODO: GPSHardware( Uart interface here)
  // TODO: GPSHardware( I2C interface here)
  ~GPSHardware();

private:
  // TODO: Add private members for managing UART OR I2C interface
};
#endif // WS_GPS_HARDWARE_H