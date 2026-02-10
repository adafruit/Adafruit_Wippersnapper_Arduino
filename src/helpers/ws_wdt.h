/*!
 * @file ws_wdt.h
 *
 * Watchdog timer wrapper class for Adafruit WipperSnapper.
 * Provides a unified interface to the Adafruit SleepyDog watchdog timer.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WS_WDT_H
#define WS_WDT_H

#include "Adafruit_SleepyDog.h"

#define WS_WDT_DEFAULT_TIMEOUT_MS 60000 ///< Default WDT timeout in milliseconds

/*!
    @brief  Watchdog timer wrapper class for WipperSnapper.
            Provides enable, disable, reconfigure, and feed operations.
*/
class ws_wdt {
public:
  ws_wdt();
  ~ws_wdt();

  int enable(int timeout_ms = WS_WDT_DEFAULT_TIMEOUT_MS);
  void disable();
  int reconfigure(int timeout_ms);
  void feed();
};

#endif // WS_WDT_H
