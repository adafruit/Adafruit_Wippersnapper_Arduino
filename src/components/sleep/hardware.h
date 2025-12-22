/*!
 * @file src/components/sleep/hardware.h
 *
 * Hardware implementation for the sleep.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SLEEP_HARDWARE_H
#define WS_SLEEP_HARDWARE_H

#ifdef ARDUINO_ARCH_ESP32
#include "Wippersnapper_V2.h"

/*!
    @brief  Interface for interacting with hardware's sleep functionality.
*/
class SleepHardware {
public:
  SleepHardware();
  ~SleepHardware();
};

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_HARDWARE_H