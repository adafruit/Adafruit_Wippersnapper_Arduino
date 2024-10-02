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
  _scale_factor = 0;
}

AnalogIOHardware::~AnalogIOHardware() {}

void AnalogIOHardware::InitPin(uint8_t pin) { pinMode(pin, INPUT); }

void AnalogIOHardware::DeinitPin(uint8_t pin) {
  pinMode(pin, INPUT); // set to a hi-z floating pin
}

void AnalogIOHardware::SetReferenceVoltage(float voltage) {
  _ref_voltage = voltage;
  _voltage_scale_factor = _ref_voltage / 65536;
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
  _adc_resolution = resolution;
  // Calculate (or re-calculate) the scale factor when we set the resolution
  CalculateScaleFactor();
}

void AnalogIOHardware::CalculateScaleFactor() {
  if (!_is_adc_resolution_scaled)
    return;

  if (_adc_resolution > _native_adc_resolution) {
    _scale_factor = _adc_resolution - _native_adc_resolution;
  } else {
    _scale_factor = _native_adc_resolution - _adc_resolution;
  }
}

uint16_t AnalogIOHardware::GetPinValue(uint8_t pin) {
  // Get the pin's value using Arduino API
  uint16_t value = analogRead(pin);
  // Scale the pins value
  if (_is_adc_resolution_scaled) {
    if (_adc_resolution > _native_adc_resolution) {
      value = value << _scale_factor;
    } else {
      value = value >> _scale_factor;
    }
  }
  return value;
}

float AnalogIOHardware::CalculatePinVoltage(uint16_t raw_value) {
  float voltage = raw_value * _voltage_scale_factor;
  return voltage;
}