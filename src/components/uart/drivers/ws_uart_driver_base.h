/*!
 * @file WipperSnapper_I2C_Driver.h
 *
 * Base implementation for UART device drivers
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

#ifndef WS_UART_DRIVER_BASE_H
#define WS_UART_DRIVER_BASE_H
#include "Wippersnapper.h"
#include <Adafruit_Sensor.h>

/**************************************************************************/
/*!
    @brief  Base class for UART Device Drivers.
*/
/**************************************************************************/
class ws_uart_driver_base {
public:
//  void ws_uart_driver_base(void) {};
//  void ~ws_uart_driver_base(void) {};
//  void configureSerial(SoftwareSerial *swSerial);
//  void configureSerial(HardwareSerial *hwSerial);
// can we just call an update() here or something and then in uart's update() we'd call the driver directly
// we added ws.h here so maybe we can try to pack and send data within sub-classes
private:
  // TODO
};
extern Wippersnapper WS;

#endif // WS_UART_DRIVER_BASE_H
