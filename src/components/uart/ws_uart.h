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

// ESP8266 platform uses SoftwareSerial
// so does RP2040 (note that this has differences from the pure softwareserial
// library, see: https://arduino-pico.readthedocs.io/en/latest/piouart.html)
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_RP2040)
#define USE_SW_UART
#include <SoftwareSerial.h>
#endif

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that provides an interface between WipperSnapper's app
            and the device's UART bus.
*/
/**************************************************************************/
class ws_ds18x20 {
public:
  // TODO

  // TODO: Constructor for using SW serial should be conditionally defined
private:
  // TODO
};
extern Wippersnapper WS;

#endif // WS_UART_H