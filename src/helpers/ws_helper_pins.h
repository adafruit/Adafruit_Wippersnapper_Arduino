/*!
 * @file src/helpers/ws_helper_pins.h
 *
 * Helper for resolving pin name strings to pin numbers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_HELPER_PINS_H
#define WS_HELPER_PINS_H
#include <Arduino.h>
#include <ctype.h>

/*!
    @brief  Resolves a pin name string to a pin number. Accepts numeric
            strings ("22"), prefixed pin names ("D22", "A4", "GPIO22"), and
            the board-defined I2C pin names ("SDA", "SCL", and "SDA1"/"SCL1"
            on boards with a second hardware I2C port).
    @param  pin_name
            The pin name string to resolve.
    @param  pin_num
            Output: the resolved pin number.
    @returns True if the pin name was resolved, False if the pin name is
             NULL, empty, or contains no pin number.
*/
inline bool WsPinNameToNum(const char *pin_name, uint32_t &pin_num) {
  if (pin_name == nullptr || pin_name[0] == '\0')
    return false;
  // Board-defined I2C pin names (CircuitPython/Blinka-style)
  if (strcmp(pin_name, "SDA") == 0) {
    pin_num = (uint32_t)SDA;
    return true;
  }
  if (strcmp(pin_name, "SCL") == 0) {
    pin_num = (uint32_t)SCL;
    return true;
  }
#if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL)
  if (strcmp(pin_name, "SDA1") == 0) {
    pin_num = (uint32_t)PIN_WIRE1_SDA;
    return true;
  }
  if (strcmp(pin_name, "SCL1") == 0) {
    pin_num = (uint32_t)PIN_WIRE1_SCL;
    return true;
  }
#endif
  // Skip a non-numeric prefix, e.g. "D22", "A4", "GPIO22"
  const char *numeric = pin_name;
  while (*numeric && !isdigit((unsigned char)*numeric))
    numeric++;
  if (*numeric == '\0')
    return false;
  pin_num = (uint32_t)atoi(numeric);
  return true;
}

#endif // WS_HELPER_PINS_H
