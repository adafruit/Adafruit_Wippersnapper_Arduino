/*!
 * @file ws_uart_drv.h
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

#ifndef WS_UART_DRV_H
#define WS_UART_DRV_H
#include "Wippersnapper.h"
#include <Adafruit_Sensor.h>

/**************************************************************************/
/*!
    @brief  Base class for UART Device Drivers.
*/
/**************************************************************************/
class ws_uart_drv {
public:
ws_uart_drv(void){};
~ws_uart_drv(void){};
// TODO! should we configure the serial within constructor or elsewhere..?
//  void configureSerial(SoftwareSerial *swSerial);
//  void configureSerial(HardwareSerial *hwSerial);
// can we just call an update() here or something and then in uart's update() we'd call the driver directly
// we added ws.h here so maybe we can try to pack and send data within sub-classes
// we'd also need to pass the sensor types wed be polling, the protos would need an update
private:
  // TODO
};
extern Wippersnapper WS;

#endif // WS_UART_DRV_H
