/*!
 * @file src/components/analogIO/hardware.cpp
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

/*!
    @brief  AnalogIO hardware constructor
*/
AnalogIOHardware::AnalogIOHardware() {
  SetNativeADCResolution(); // Configure the device's native ADC resolution
  SetResolution(
      DEFAULT_ADC_RESOLUTION); // Wippersnapper's default resolution is 16-bit
  SetReferenceVoltage(
      DEFAULT_REF_VOLTAGE); // Wippersnapper's default ref voltage is 3.3V
}

/*!
    @brief  AnalogIO hardware destructor
*/
AnalogIOHardware::~AnalogIOHardware() {}

/*!
    @brief  Initializes an analog input pin.
    @param  pin
            The requested pin.
*/
void AnalogIOHardware::InitPin(uint8_t pin) { pinMode(pin, INPUT); }

/*!
    @brief  Deinitializes an analog input pin and frees it for
            other uses.
    @param  pin
            The requested pin.
*/
void AnalogIOHardware::DeinitPin(uint8_t pin) {
  pinMode(pin, INPUT); // set to a hi-z floating pin
}

/*!
    @brief  Sets the hardware's reference voltage for reading analog pins
    @param  voltage
            The reference voltage.
*/
void AnalogIOHardware::SetReferenceVoltage(float voltage) {
  _ref_voltage = voltage;
}

/*!
    @brief  Configures the hardware's native ADC resolution.
*/
void AnalogIOHardware::SetNativeADCResolution() {
#if defined(ARDUINO_ARCH_SAMD)
  _native_adc_resolution = 12;
#elif defined(ARDUINO_ARCH_ESP32)
#if defined(CONFIG_IDF_TARGET_ESP32S3)
  // In arduino-esp32, the ESP32-S3 ADC uses a 13-bit resolution
  _native_adc_resolution = 13;
#else
  // The ESP32, ESP32-S2, ESP32-C3 ADCs uses 12-bit resolution
  _native_adc_resolution = 12;
#endif
#elif defined(ARDUINO_ARCH_RP2040)
  _native_adc_resolution = 10;
#else
  _native_adc_resolution = 10;
#endif
#ifndef ARDUINO_ARCH_ESP8266
  // Set the resolution (in bits) of the hardware's ADC
  analogReadResolution(_native_adc_resolution);
#endif // ARDUINO_ARCH_ESP8266
}

/*!
    @brief  Sets the hardware ADC's resolution.
    @param  resolution
            The requested resolution, in bits.
*/
void AnalogIOHardware::SetResolution(uint8_t resolution) {
  _desired_adc_resolution = resolution;
  // Calculate (or re-calculate) the scale factor whenever we set the desired
  // read resolution
  CalculateScaleFactor();
}

/*!
    @brief  Calculates a factor used by the hardware for scaling
            the ADC's resolution.
*/
void AnalogIOHardware::CalculateScaleFactor() {
  _max_scale_resolution_desired =
      pow(2, 16); // TODO: Change to _desired_adc_resolution
  _max_scale_resolution_native =
      pow(2, 13); // TODO: Change to _native_adc_resolution
}

/*!
    @brief  Gets the "raw" value of an analog pin from the ADC.
    @param  pin
            The requested pin.
    @return The pin's value.
*/
uint16_t AnalogIOHardware::GetPinValue(uint8_t pin) {
  return (analogRead(pin) * _max_scale_resolution_desired) /
         _max_scale_resolution_native;
}

/*!
    @brief  Gets the voltage of an analog pin.
    @param  pin
            The requested pin.
    @return The pin's voltage.
*/
float AnalogIOHardware::GetPinVoltage(uint8_t pin) {
#ifdef ARDUINO_ARCH_ESP32
  return analogReadMilliVolts(pin) / 1000.0;
#else
  return (GetPinValue(pin) * _ref_voltage) / 65536;
#endif // ARDUINO_ARCH_ESP32
}