/*!
 * @file model.h
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

/**************************************************************************/
/*!
    @brief  Interface for interacting with hardware's analog i/o pin API.
*/
/**************************************************************************/
class AnalogIOHardware {
public:
  AnalogIOHardware();
  ~AnalogIOHardware();
  void SetNativeADCResolution();
  void SetResolution(uint8_t resolution);
  uint8_t GetResolution(void);

  void InitPin(uint8_t pin);
private:
  uint8_t _native_adc_resolution;
  uint8_t _adc_resolution;
  bool _is_adc_resolution_scaled;
};
#endif // WS_ANALOGIO_HARDWARE_H