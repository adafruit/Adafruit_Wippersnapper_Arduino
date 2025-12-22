/*!
 * @file src/components/sleep/hardware.cpp
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
#ifdef ARDUINO_ARCH_ESP32
#include "hardware.h"

/*!
    @brief  Sleep hardware constructor
*/
SleepHardware::SleepHardware() {
}

/*!
    @brief  Sleep hardware destructor
*/
SleepHardware::~SleepHardware() {
}

#endif // ARDUINO_ARCH_ESP32