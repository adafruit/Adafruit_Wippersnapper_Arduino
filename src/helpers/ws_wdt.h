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
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
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
// Advanced Sleep API for RP2350
#ifdef ARDUINO_ARCH_RP2350
  void registerTimerWakeup(int max_period_ms = 0);
  void registerGPIOWakeup(uint gpio_pin, bool edge = false, bool high = false);
  void startSleep();
  long getSleepDuration();
  void resumeFromSleep();
  bool didWakeFromSleep();
  bool isSleepConfigTimer();
#endif
private:
#ifdef ARDUINO_ARCH_RP2350
  bool
      _did_wake_from_sleep; ///< True if device woke from sleep, False otherwise
  bool _is_sleep_cfg_timer; ///< True if timer-based sleep, False if GPIO-based
  int _max_sleep_period_ms; ///< Maximum sleep period in milliseconds
  uint _sleep_gpio_pin;     ///< Desired GPIO pin number for wakeup
  bool _sleep_gpio_edge;    ///< Specifies whether to wake on edge or level
  bool _sleep_gpio_level; ///< Specifies whether to wake on a high or low level
  static volatile bool _awake; ///< Static flag for wake callback
  static void
  wakeCallback(); ///< Static callback for RP2350 Watchdog.setWakeCb()
#endif
};

#endif // WS_WDT_H
