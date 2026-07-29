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
#include <stdlib.h>

#define WS_PIN_INVALID -1 ///< Returned when a pin name fails to resolve

/*!
    @brief  Parses an expander pin name ("EXP_0xNN_P") and returns the
            expander's I2C address. Use this to detect expander pins and
            to look up the owning driver, e.g. via
            ExpanderController::GetDriver().
    @param  pin_name
            The pin name string to parse.
    @returns The expander's I2C address, or WS_PIN_INVALID if the name is
             not a well-formed expander pin name.
*/
inline int32_t WsPinNameToExpanderAddr(const char *pin_name) {
  if (pin_name == nullptr || strncmp(pin_name, "EXP_", 4) != 0)
    return WS_PIN_INVALID;
  char *end = nullptr;
  unsigned long addr = strtoul(pin_name + 4, &end, 16);
  if (end == pin_name + 4 || *end != '_' || addr > 0x7F)
    return WS_PIN_INVALID;
  return (int32_t)addr;
}

/*!
    @brief  Resolves a pin name string to a pin number. Accepts numeric
            strings ("22"), prefixed pin names ("D22", "A4", "GPIO22"), the
            board-defined I2C pin names ("SDA", "SCL", and "SDA1"/"SCL1"
            on boards with a second hardware I2C port), and expander pin
            names ("EXP_0x48_3"). For expander pins the returned number is
            the pin (channel) index on that expander — e.g. channels 0-7
            and spare GPIO 8-15 on a 16-channel driver — scoped to the
            expander identified by WsPinNameToExpanderAddr(), not a native
            MCU pin.
    @param  pin_name
            The pin name string to resolve.
    @returns The resolved pin number, or WS_PIN_INVALID if the pin name is
             NULL, empty, malformed, or contains no pin number.
*/
inline int32_t WsPinNameToNum(const char *pin_name) {
  if (pin_name == nullptr || pin_name[0] == '\0')
    return WS_PIN_INVALID;
  // Expander pin names ("EXP_0xNN_P") resolve to the pin index on the
  // expander at address 0xNN
  if (strncmp(pin_name, "EXP_", 4) == 0) {
    if (WsPinNameToExpanderAddr(pin_name) == WS_PIN_INVALID)
      return WS_PIN_INVALID;
    const char *pin_str = strchr(pin_name + 4, '_');
    if (pin_str == nullptr || !isdigit((unsigned char)pin_str[1]))
      return WS_PIN_INVALID;
    return (int32_t)atoi(pin_str + 1);
  }
  // Board-defined I2C pin names (CircuitPython/Blinka-style)
  if (strcmp(pin_name, "SDA") == 0)
    return (int32_t)SDA;
  if (strcmp(pin_name, "SCL") == 0)
    return (int32_t)SCL;
#if defined(PIN_WIRE1_SDA) && defined(PIN_WIRE1_SCL)
  if (strcmp(pin_name, "SDA1") == 0)
    return (int32_t)PIN_WIRE1_SDA;
  if (strcmp(pin_name, "SCL1") == 0)
    return (int32_t)PIN_WIRE1_SCL;
#elif defined(SDA1) && defined(SCL1)
  if (strcmp(pin_name, "SDA1") == 0)
    return (int32_t)SDA1;
  if (strcmp(pin_name, "SCL1") == 0)
    return (int32_t)SCL1;
#endif
  // Skip a non-numeric prefix, e.g. "D22", "A4", "GPIO22"
  const char *numeric = pin_name;
  while (*numeric && !isdigit((unsigned char)*numeric))
    numeric++;
  if (*numeric == '\0')
    return WS_PIN_INVALID;
  return (int32_t)atoi(numeric);
}

/*!
    @brief  A pin reference: pairs the broker-facing pin name string with
            its resolved pin number, so the name is validated and resolved
            exactly once and both representations travel together instead
            of re-resolving the string at every use.
*/
struct WsPinName {
  char name[32] = {0}; ///< Pin name string, sized to hold any pb pin-name
                       ///< field (holders static_assert this against the
                       ///< pb field sizes they mirror).
  int32_t num = WS_PIN_INVALID; ///< Resolved pin number, or WS_PIN_INVALID.

  /*!
      @brief  Validates a pin name and resolves it to a pin number.
      @param  pin_name
              The pin name string to store and resolve.
      @returns True if pin_name is non-null, fits the buffer without
               truncation, and resolves to a pin number; False otherwise
               (fields are reset to empty / WS_PIN_INVALID).
  */
  bool Set(const char *pin_name) {
    name[0] = '\0';
    num = WS_PIN_INVALID;
    if (pin_name == nullptr)
      return false;
    if (strlen(pin_name) >= sizeof(name))
      return false; // would truncate
    strncpy(name, pin_name, sizeof(name) - 1);
    name[sizeof(name) - 1] = '\0';
    num = WsPinNameToNum(name);
    return num != WS_PIN_INVALID;
  }
};

#endif // WS_HELPER_PINS_H
