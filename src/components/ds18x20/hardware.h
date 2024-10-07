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
#include <DallasTemperature.h>

struct DS18X20_Pin {
  // Specific to the DS18X20 sensor object
  OneWire
      *oneWire; ///< Ptr reference to the OneWire bus object used by this pin
  DallasTemperature
      *dallasTempObj; ///< Pointer to a DallasTemperature sensor object
  DeviceAddress dallasTempAddr; ///< Temperature sensor's address
  // From the PB model
  char onewire_pin[5]; ///< Pin utilized by the OneWire bus, used for addressing
  float period;        ///< The desired period to read the sensor, in seconds
  float prv_period;    ///< Last time the sensor was polled, in seconds
  pb_size_t
      sensor_types_count; ///< Number of sensor types to read from the sensor
  wippersnapper_sensor_SensorType
      sensor_types[2]; ///< DS sensor type(s) to read from the sensor
};                     ///< DS18X20 Pin Object

/**************************************************************************/
/*!
    @brief  Interface for interacting with the's DallasTemp
            and OneWire APIs.
*/
/**************************************************************************/
class DS18X20Hardware {
public:
  DS18X20Hardware();
  ~DS18X20Hardware();

private:
  std::vector<DS18X20_Pin> _DS18X20_Pins; ///< Vector of analogio pins
};
#endif // WS_DS18X20_HARDWARE_H