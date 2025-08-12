/*!
 * @file src/components/analogIO/hardware.h
 *
 * Hardware implementation for the analogio.proto message.
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
#ifndef WS_ANALOGIO_HARDWARE_H
#define WS_ANALOGIO_HARDWARE_H
#include "Wippersnapper_V2.h"

/*!
    @brief  Interface for interacting with hardware's analog i/o pin API.
*/
class AnalogIOHardware {
public:
  AnalogIOHardware();
  ~AnalogIOHardware();
  void SetNativeADCResolution();
  void SetResolution(uint8_t resolution);
  void SetReferenceVoltage(float voltage);
  void CalculateScaleFactor();
  // Arduino/Wiring API
  void InitPin(uint8_t pin);
  void DeinitPin(uint8_t pin);
  uint16_t GetPinValue(uint8_t pin);
  float GetPinVoltage(uint8_t pin);

private:
  uint8_t _native_adc_resolution;    ///< Hardware's native ADC resolution.
  uint8_t _desired_adc_resolution;   ///< Desired (final) ADC resolution.
  int _max_scale_resolution_desired; ///< Maximum scale resolution desired.
  int _max_scale_resolution_native;  ///< Maximum scale resolution native.

  bool _is_adc_resolution_scaled; ///< True if the ADC's resolution is scaled,
                                  ///< False otherwise.
  float _ref_voltage; ///< Reference voltage for reading analog pins.
};
#endif // WS_ANALOGIO_HARDWARE_H