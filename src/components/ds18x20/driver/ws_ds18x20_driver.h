/*!
 * @file ws_ds18x20_driver.h
 *
 * Device driver for a DS18x20 Sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef ws_ds18x20_driver
#define ws_ds18x20_driver

#include "ws_ds18x20.h"

#ifdef ARDUINO_ARCH_ESP32
// PaulStoffregen OneWire is incompatable with arduino-esp32 v2.0.1
// see: https://github.com/PaulStoffregen/OneWire/issues/112 and
// https://github.com/espressif/arduino-esp32/issues
#include <OneWireNg.h>
#else
// Non-arduino-esp32 arch. should use PaulStoffregen OneWire
#include <OneWire.h>
#endif

#include <Adafruit_Sensor.h>
#include <DallasTemperature.h>

/**************************************************************************/
/*!
    @brief  TODO
*/
/**************************************************************************/
class ws_ds18x20_driver  {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an AHTX0 sensor.
  */
  /*******************************************************************************/
  ws_ds18x20_driver::ws_ds18x20_driver() {
    // TODO
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AHTX0 sensor.
  */
  /*******************************************************************************/
  ~ws_ds18x20_driver() { // TODO }

  /*******************************************************************************/
  /*!
      @brief    Initializes the DS18x20 sensor and begins communication.
      @returns  True if initialized successfully, False otherwise.

  */
  /*******************************************************************************/
  bool begin() {
/*     // attempt
    _aht = new Adafruit_AHTX0();
    if (!_aht->begin(_i2c, (int32_t)_sensorAddress))
      return false;

    // get temperature and humidity sensor
    _aht_temp = _aht->getTemperatureSensor();
    _aht_humidity = _aht->getHumiditySensor(); */

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
/*   bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // is sensor enabled correctly?
    if (_aht_temp == NULL)
      return false;
    // get event
    _aht_temp->getEvent(tempEvent);
    return true;
  } */

  /*******************************************************************************/
  /*!
      @brief    Gets the AHTX0's current humidity.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
/*   bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // is sensor enabled correctly?
    if (_aht_humidity == NULL)
      return false;
    // get humidity
    _aht_humidity->getEvent(humidEvent);
    return true;
  } */

protected:
/*   Adafruit_AHTX0 *_aht; ///< Pointer to an AHTX0 object
  Adafruit_Sensor *_aht_temp =
      NULL; ///< Holds data for the AHTX0's temperature sensor
  Adafruit_Sensor *_aht_humidity =
      NULL; ///< Holds data for the AHTX0's humidity sensor */
};

#endif // ws_ds18x20_driver