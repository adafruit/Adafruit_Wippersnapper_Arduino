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


class Wippersnapper; // forward declaration

/**************************************************************************/
/*!
    @brief  Class that provides an interface between WipperSnapper's app
            and the device's UART bus.
*/
/**************************************************************************/
class ws_uart {
public:
  ws_uart(wippersnapper_uart_v1_UARTDeviceAttachRequest *msgUARTRequest);
  ~ws_uart(void);

  // TODO: Constructor for using SW serial should be conditionally defined
private:
#ifdef USE_SW_UART
  SoftwareSerial *_swSerial = nullptr;
#else
  HardwareSerial *_hwSerial = nullptr;
#endif

};
extern Wippersnapper WS;

#endif // WS_UART_H