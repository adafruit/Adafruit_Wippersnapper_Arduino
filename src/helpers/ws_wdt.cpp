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
ws_wdt::ws_wdt() {
#ifdef ARDUINO_ARCH_RP2350
  _did_wake_from_sleep = false;
  _is_sleep_cfg_timer = false;
  _max_sleep_period_ms = 0;
  _sleep_gpio_pin = 0;
  _sleep_gpio_edge = false;
  _sleep_gpio_level = false;
#endif
}

/*!
    @brief  Destructor for the watchdog timer wrapper class.
*/
ws_wdt::~ws_wdt() {
#ifdef ARDUINO_ARCH_RP2350
  _did_wake_from_sleep = false;
  _is_sleep_cfg_timer = false;
  _max_sleep_period_ms = 0;
  _sleep_gpio_pin = 0;
  _sleep_gpio_edge = false;
  _sleep_gpio_level = false;
#endif
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

// Advanced Sleep API for RP2350
#ifdef ARDUINO_ARCH_RP2350

/*!
    @brief  Registers an AON timer wakeup source for the RP2350.
    @param  max_period_ms
            The maximum sleep duration in milliseconds. If 0, sleeps
   indefinitely until an external wake event occurs.
*/
void ws_wdt::registerTimerWakeup(int max_period_ms) {
  _did_wake_from_sleep = false;
  _is_sleep_cfg_timer = true;
  _max_sleep_period_ms = max_period_ms;
}

/*!
    @brief  Registers a GPIO pin wakeup source for the RP2350.
    @param  gpio_pin
            The GPIO pin number to monitor for the wake event.
    @param  edge
            If true, wake on any change (rising or falling edge). If false, wake
            on the specified level.
    @param  high
            If edge is false, specifies whether to wake on a high level (true)
            or low level (false) on the GPIO pin.
*/
void ws_wdt::registerGPIOWakeup(uint gpio_pin, bool edge, bool high) {
  _did_wake_from_sleep = false;
  _is_sleep_cfg_timer = false;
  _sleep_gpio_pin = gpio_pin;
  _sleep_gpio_edge = edge;
  _sleep_gpio_level = high;
}

/*!
    @brief  Starts sleep mode based on the registered wakeup configuration.
*/
void ws_wdt::startSleep() {
  Watchdog.goToSleepUntil(5000, true);
/*   if (_is_sleep_cfg_timer) {
    Watchdog.goToSleepUntil(5000, true);
  } else {
    Watchdog.goToSleepUntilPin(_sleep_gpio_pin, _sleep_gpio_edge,
                               _sleep_gpio_level);
  } */
}

/*!
    @brief  Gets the duration of the most recent sleep period for RP2350.
    @returns The sleep duration in milliseconds.
*/
long ws_wdt::getSleepDuration() { return Watchdog.getSleepDuration(); }

/*!
    @brief  Resumes the RP2350 from sleep mode. Should be called after waking up
            to properly reset the WDT state.
*/
void ws_wdt::resumeFromSleep() {
  _did_wake_from_sleep = true;
  Watchdog.resumeFromSleep();
}

/*!
    @brief  Checks if the sleep configuration is timer-based.
    @returns True if timer-based, False if GPIO-based.
*/
bool ws_wdt::isSleepConfigTimer() { return _is_sleep_cfg_timer; }

/*!
    @brief  Checks if the RP2350 woke from sleep mode.
    @returns True if the device woke from sleep, False otherwise.
*/
bool ws_wdt::didWakeFromSleep() { return _did_wake_from_sleep; }

#endif