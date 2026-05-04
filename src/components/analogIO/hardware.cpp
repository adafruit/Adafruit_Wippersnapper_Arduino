/*!
 * @file src/components/analogIO/hardware.cpp
 *
 * Hardware driver for the analogio.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "hardware.h"

/*!
    @brief  AnalogIOHardware constructor
    @param  pin_name      The pin number.
    @param  read_mode     The type of analog read (RAW or VOLTAGE).
    @param  sample_mode   The pin's sample mode (TIMER or EVENT).
    @param  period        The pin's period in milliseconds.
    @param  ref_voltage   The reference voltage for analog reads.
    @param  expander_drv  Pointer to expander driver, or nullptr.
*/
AnalogIOHardware::AnalogIOHardware(uint8_t pin_name, ws_sensor_Type read_mode,
                                   ws_analogio_SampleMode sample_mode,
                                   ulong period, float ref_voltage,
                                   ExpanderHardware *expander_drv)
    : _name(pin_name), _read_mode(read_mode), _sample_mode(sample_mode),
      _period(period), _prv_time(0), _did_read_send(false), _value_raw(0),
      _value_voltage(0.0f), _prv_value_raw(0), _native_adc_resolution(0),
      _desired_adc_resolution(0), _max_scale_resolution_desired(0),
      _max_scale_resolution_native(0), _mcu_vref(ref_voltage),
      _expander_drv(expander_drv) {
  SetNativeADCResolution();
  // TODO: When we have expanders in, look at this again!
  SetResolution(DEFAULT_ADC_RESOLUTION);
  InitPin();
}

/*!
    @brief  AnalogIOHardware destructor. Resets pin to floating INPUT state.
*/
AnalogIOHardware::~AnalogIOHardware() { DeinitPin(); }

/*!
    @brief  Initializes an analog input pin.
*/
void AnalogIOHardware::InitPin() { pinMode(_name, INPUT); }

/*!
    @brief  Deinitializes an analog input pin and frees it for
            other uses.
*/
void AnalogIOHardware::DeinitPin() {
  pinMode(_name, INPUT); // set to a hi-z floating pin
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
  _max_scale_resolution_desired = pow(2, _desired_adc_resolution);
  _max_scale_resolution_native = pow(2, _native_adc_resolution);
}

/*!
    @brief  Reads the raw ADC value from the pin, scaled to the
            desired resolution.
    @return The raw ADC value.
*/
uint16_t AnalogIOHardware::ReadRawValue() {
  _value_raw = (analogRead(_name) * _max_scale_resolution_desired) /
               _max_scale_resolution_native;
  return _value_raw;
}

/*!
    @brief  Reads the voltage from the analog pin.
    @return The pin's voltage.
*/
float AnalogIOHardware::ReadVoltage() {
#ifdef ARDUINO_ARCH_ESP32
  _value_voltage = analogReadMilliVolts(_name) / 1000.0;
#else
  _value_voltage = (ReadRawValue() * _mcu_vref) / 65536;
#endif // ARDUINO_ARCH_ESP32
  return _value_voltage;
}

/*!
    @brief  Checks if the pin's raw value has changed since last check.
    @return True if the value changed.
*/
bool AnalogIOHardware::CheckEvent() {
  ReadRawValue();

  if (_value_raw == _prv_value_raw)
    return false;
  _prv_value_raw = _value_raw;
  return true;
}

/*!
    @brief  Reads the pin according to its read_mode.
    @return The pin's value as a float (raw cast or voltage).
*/
float AnalogIOHardware::ReadValue() {
  if (_read_mode == ws_sensor_Type_T_RAW) {
    return (float)ReadRawValue();
  }
  return ReadVoltage();
}

/*!
    @brief  Checks if the pin's timer has expired and reads the value.
    @return True if the timer expired.
*/
bool AnalogIOHardware::CheckTimer() {
  ulong cur_time = millis();
  if (cur_time - _prv_time <= _period)
    return false;
  _prv_time = cur_time;
  return true;
}

/*!
    @brief  Gets the pin's number.
    @return The pin's number.
*/
uint8_t AnalogIOHardware::GetPinNum() const { return _name; }

/*!
    @brief  Gets the pin's read mode.
    @return The pin's read mode (RAW or VOLTAGE).
*/
ws_sensor_Type AnalogIOHardware::GetReadMode() const { return _read_mode; }

/*!
    @brief  Gets the pin's sample mode.
    @return The pin's sample mode (TIMER or EVENT).
*/
ws_analogio_SampleMode AnalogIOHardware::GetSampleMode() const {
  return _sample_mode;
}

/*!
    @brief  Gets the last read value according to read_mode.
    @return The last value as a float (raw cast or voltage).
*/
float AnalogIOHardware::GetValue() const {
  if (_read_mode == ws_sensor_Type_T_RAW) {
    return (float)_value_raw;
  }
  return _value_voltage;
}

/*!
    @brief  Gets whether the last read was sent to IO.
    @return True if the last read was sent successfully.
*/
bool AnalogIOHardware::DidReadSend() const { return _did_read_send; }

/*!
    @brief  Marks that the current pin value has been sent to IO.
*/
void AnalogIOHardware::MarkSent() { _did_read_send = true; }

/*!
    @brief  Resets the pin's did_read_send flag to false.
*/
void AnalogIOHardware::ResetSendFlag() { _did_read_send = false; }
