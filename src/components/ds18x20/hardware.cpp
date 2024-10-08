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
  // Initialize the OneWire bus object
  new (&_ow) OneWireNg_CurrentPlatform(onewire_pin, false);
}

DS18X20Hardware::~DS18X20Hardware() { delete &_ow; }

bool DS18X20Hardware::GetSensor() {
  OneWireNg::ErrorCode ec = _ow->readSingleId(_sensorId);
  return ec == OneWireNg::EC_SUCCESS;
}

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

bool DS18X20Hardware::IsTimerExpired() {
  // Get the current time in milliseconds and compare it to the last time
  // the sensor was polled
  ulong cur_time = millis();
  return millis() - _prv_period > _period;
}

float DS18X20Hardware::GetTemperatureC() { return _temp_c; }

float DS18X20Hardware::GetTemperatureF() { return _temp_f; }

bool DS18X20Hardware::ReadTemperatureF() {
  bool is_success = ReadTemperatureC();
  // Did we read the temperature successfully?
  if (!is_success)
    return false;
  // We have the temperature in Celsius, convert to Fahrenheit
  _temp_f = _temp_c * 9.0 / 5.0 + 32.0;
  return true;
}

bool DS18X20Hardware::ReadTemperatureC() {
  // Start temperature conversion for the first identified sensor on the OneWire
  // bus
  OneWireNg::ErrorCode ec =
      _drv_therm.convertTemp(_sensorId, DSTherm::MAX_CONV_TIME, false);
  if (ec != OneWireNg::EC_SUCCESS)
    return false

        // Scratchpad placeholder is static to allow reuse of the associated
        // sensor id while reissuing readScratchpadSingle() calls.
        // Note, due to its storage class the placeholder is zero initialized.
        static Placeholder<DSTherm::Scratchpad>
            scrpd;
  ec = _drv_therm.readScratchpadSingle(scrpd);
  if (ec != OneWireNg::EC_SUCCESS)
    return false;

  // Read the temperature from the sensor
  long temp = scrpd->getTemp2();
  _temp_c = temp / 16.0; // Convert from 16-bit int to float
  return true;
}