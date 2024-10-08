/*!
 * @file hardware.cpp
 *
 * Hardware interface for the ds18x20.proto API
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
#include "hardware.h"

DS18X20Hardware::DS18X20Hardware(uint8_t onewire_pin) : _drv_therm(_ow) {
  new (&_ow) OneWireNg_CurrentPlatform(onewire_pin, false);
}

DS18X20Hardware::~DS18X20Hardware() { delete &_ow; }

void DS18X20Hardware::setResolution(int resolution) {
  // Set the resolution of the DS18X20 sensor driver
  switch (resolution) {
  case 9:
    _resolution = DSTherm::Resolution::RES_9_BIT;
    break;
  case 10:
    _resolution = DSTherm::Resolution::RES_10_BIT;
    break;
  case 11:
    _resolution = DSTherm::Resolution::RES_11_BIT;
    break;
  case 12:
    _resolution = DSTherm::Resolution::RES_12_BIT;
    break;
  default:
    _resolution =
        DSTherm::Resolution::RES_12_BIT; // Default to 12-bit resolution
    break;
  }

  // Set common resolution for all sensors.
  // Th, Tl (high/low alarm triggers) are set to 0.
  _drv_therm.writeScratchpadAll(0, 0, _resolution);

  // The configuration above is stored in volatile sensors scratchpad
  // memory and will be lost after power unplug. Therefore store the
  // configuration permanently in sensors EEPROM.
  _drv_therm.copyScratchpadAll(false);
}