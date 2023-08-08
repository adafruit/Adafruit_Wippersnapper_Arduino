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

// ESP8266 platform uses SoftwareSerial
// so does RP2040 (note that this has differences from the pure softwareserial
// library, see: https://arduino-pico.readthedocs.io/en/latest/piouart.html)
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_RP2040)
#define USE_SW_UART
#include <SoftwareSerial.h>
#else
#include <HardwareSerial.h>
#endif

/**************************************************************************/
/*!
    @brief  Base class for UART Device Drivers.
*/
/**************************************************************************/
class ws_uart_drv {
public:
#ifdef USE_SW_UART
  ws_uart_drv(SoftwareSerial *swSerial, int32_t pollingInterval){};
#else
  ws_uart_drv(HardwareSerial *hwSerial, int32_t pollingInterval){};
#endif

  /*******************************************************************************/
  /*!
      @brief    Destructor for a UART device driver.
  */
  /*******************************************************************************/
  ~ws_uart_drv(void){};

  // TODO:
  // can we just call an update() here or something and then in uart's update()
  // we'd call the driver directly we added ws.h here so maybe we can try to
  // pack and send data within sub-classes we'd also need to pass the sensor
  // types wed be polling, the protos would need an update
  int32_t pollingInterval; ///< UART device's polling interval, in milliseconds
private:
};

#endif // WS_UART_DRV_H
