/*!
 * @file hardware.cpp
 *
 * Hardware interface for the analogio.proto API
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

AnalogIOHardware::AnalogIOHardware() {
  SetNativeADCResolution(); // Configure the device's native ADC resolution
}

AnalogIOHardware::~AnalogIOHardware() {}

void AnalogIOHardware::InitPin(uint8_t pin) {
    pinMode(pin, INPUT);
}

void AnalogIOHardware::SetNativeADCResolution() {
_is_adc_resolution_scaled = false;
#if defined(ARDUINO_ARCH_SAMD)
  analogReadResolution(16);
  _native_adc_resolution = 12;
#elif defined(ARDUINO_ARCH_ESP32)
  _is_adc_resolution_scaled = true;
  _native_adc_resolution = 13;
#elif defined(ARDUINO_ARCH_RP2040)
  _is_adc_resolution_scaled = true;
  _native_adc_resolution = 10;
#else
  _is_adc_resolution_scaled = true;
  _native_adc_resolution = 10;
#endif
}

void AnalogIOHardware::SetResolution(uint8_t resolution) {
  _adcResolution = resolution;
}


uint8_t AnalogIOHardware::GetResolution(void) {
    return _adc_resolution;
}