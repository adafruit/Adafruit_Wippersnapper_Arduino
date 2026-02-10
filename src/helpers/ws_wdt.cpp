/*!
 * @file ws_wdt.cpp
 *
 * Watchdog timer wrapper class implementation for Adafruit WipperSnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "ws_wdt.h"

/*!
    @brief  Constructor for the watchdog timer wrapper class.
*/
ws_wdt::ws_wdt() {}

/*!
    @brief  Destructor for the watchdog timer wrapper class.
*/
ws_wdt::~ws_wdt() {

}

/*!
    @brief  Enables the watchdog timer with the specified timeout.
    @param  timeout_ms
            The timeout duration in milliseconds. Defaults to
            WS_WDT_DEFAULT_TIMEOUT_MS (60000ms).
    @returns The actual timeout set by the hardware (may differ from requested),
             or 0 on failure.
*/
int ws_wdt::enable(int timeout_ms) {
#ifdef OFFLINE_MODE_WOKWI
  // Wokwi simulator does not support WDT
  _enabled = true;
  _timeout_ms = timeout_ms;
  return timeout_ms;
#else
  int rc = Watchdog.enable(timeout_ms);
  return rc;
#endif
}

/*!
    @brief  Disables the watchdog timer.
*/
void ws_wdt::disable() {
#ifndef OFFLINE_MODE_WOKWI
#ifndef ARDUINO_ARCH_RP2040
  // RP2040 WDT cannot be disabled once enabled
  Watchdog.disable();
#endif
#endif
}

/*!
    @brief  Reconfigures the watchdog timer with a new timeout.
    @param  timeout_ms
            The new timeout duration in milliseconds.
    @returns The actual timeout set by the hardware, or 0 on failure.
    @note   On RP2040, the WDT cannot be reconfigured after enabling,
            so this will return 0.
*/
int ws_wdt::reconfigure(int timeout_ms) {
#ifdef ARDUINO_ARCH_RP2040
  // RP2040 WDT cannot be reconfigured after enabling
  return 0;
#endif

#ifdef OFFLINE_MODE_WOKWI
  _timeout_ms = timeout_ms;
  return timeout_ms;
#else
  Watchdog.disable();
  int rc = Watchdog.enable(timeout_ms);
  return rc;
#endif
}

/*!
    @brief  Feeds (resets) the watchdog timer to prevent a hardware reset.
*/
void ws_wdt::feed() {
#ifndef OFFLINE_MODE_WOKWI
  Watchdog.reset();
#endif
}
