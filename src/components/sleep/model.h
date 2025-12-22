/*!
 * @file src/components/sleep/model.h
 *
 * Model interface for the sleep.proto message.
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
#ifndef WS_SLEEP_MODEL_H
#define WS_SLEEP_MODEL_H

#ifdef ARDUINO_ARCH_ESP32
#include "Wippersnapper_V2.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from sleep.proto.
*/
class SleepModel {
public:
  SleepModel();
  ~SleepModel();
};

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_MODEL_H