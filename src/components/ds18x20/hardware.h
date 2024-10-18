/*!
 * @file model.h
 *
 * Hardware implementation for the ds18x20.proto message.
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
#ifndef WS_DS18X20_HARDWARE_H
#define WS_DS18X20_HARDWARE_H
#include "Wippersnapper_V2.h"

#include "OneWireNg_CurrentPlatform.h"
#include "drivers/DSTherm.h"
#include "utils/Placeholder.h"

/**************************************************************************/
/*!
    @brief  Interface for interacting with the's DallasTemp
            and OneWire APIs.
*/
/**************************************************************************/
class DS18X20Hardware {
public:
  DS18X20Hardware(uint8_t onewire_pin);
  ~DS18X20Hardware();
  uint8_t GetOneWirePin();
  void SetResolution(int resolution);
  void SetPeriod(float period);
  void setOneWirePinName(const char *prettyOWPinName);
  const char *getOneWirePinName();
  bool IsTimerExpired();
  bool GetSensor();
  bool ReadTemperatureC();
  float GetTemperatureC();
  float GetTemperatureF();
  bool is_read_temp_c; ///< Flag telling the controller to read the temperature
                       ///< in degrees Celsius
  bool is_read_temp_f; ///< Flag telling the controller to read the temperature
                       ///< in degrees Fahrenheit
private:
  Placeholder<OneWireNg_CurrentPlatform> _ow; ///< OneWire bus object
  OneWireNg::Id _sensorId;                    ///< Sensor ID
  DSTherm _drv_therm;                         ///< DS18X20 driver object
  DSTherm::Resolution _resolution; ///< Resolution of the DS18X20 sensor
  float _temp_c;                   ///< Temperature in Celsius
  float _temp_f;                   ///< Temperature in Fahrenheit
  // From the PB model
  uint8_t
      _onewire_pin; ///< Pin utilized by the OneWire bus, used for addressing
  char _onewire_pin_name[5]; ///< Name of the OneWire bus pin
  float _period;     ///< The desired period to read the sensor, in seconds
  float _prv_period; ///< Last time the sensor was polled, in seconds
};
#endif // WS_DS18X20_HARDWARE_H