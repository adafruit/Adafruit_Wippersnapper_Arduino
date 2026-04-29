/*!
 * @file src/components/digitalIO/hardware.h
 *
 * Hardware for the digitalio.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_DIGITALIO_HARDWARE_H
#define WS_DIGITALIO_HARDWARE_H
#include "wippersnapper.h"

struct DigitalIOPin; // Forward declaration

/*!
    @brief  Interface for interacting with hardware's digital I/O pin API.
*/
class DigitalIOHardware {
public:
  DigitalIOHardware();
  ~DigitalIOHardware();
  bool SetPinMode(DigitalIOPin *pin);
  void SetValue(DigitalIOPin *pin, bool pin_value);
  bool GetValue(DigitalIOPin *pin);
  void deinit(DigitalIOPin *pin);
  bool IsStatusLEDPin(DigitalIOPin *pin);

private:
};
#endif // WS_DIGITALIO_HARDWARE_H