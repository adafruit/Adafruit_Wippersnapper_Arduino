/*!
 * @file WipperSnapper_I2C_Driver_SPA06_003.h
 *
 * Device driver for an SPA06-003 Pressure and Temperature sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_SPA06_003_H
#define WipperSnapper_I2C_Driver_SPA06_003_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_SPA06_003.h>

#define SPA06_003_TEMP_MIN -40.0 ///< Minimum valid temperature reading
#define SPA06_003_TEMP_MAX 85.0  ///< Maximum valid temperature reading
#define SPA06_003_PRESSURE_MIN                                                 \
  300.0 ///< Minimum valid pressure (9km above sea level)
#define SPA06_003_PRESSURE_MAX                                                 \
  1100.0 ///< Maximum valid pressure (500m below sea level)

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the SPA06-003 PT sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SPA06_003 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an SPA06-003 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SPA06_003(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an SPA06-003 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_SPA06_003() { delete _spa06_003; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SPA06-003 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _spa06_003 = new Adafruit_SPA06_003();
    // attempt to initialize SPA06-003
    if (!_spa06_003->begin(_sensorAddress, _i2c))
      return false;

    _spa06_003_temp = _spa06_003->getTemperatureSensor();
    _spa06_003_pressure = _spa06_003->getPressureSensor();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SPA06-003's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    bool success = false;
    if (_spa06_003_temp == NULL)
      return false;
    success = _spa06_003_temp->getEvent(tempEvent);
    if (tempEvent->temperature > SPA06_003_TEMP_MAX ||
        tempEvent->temperature < SPA06_003_TEMP_MIN) {
      success = false;
    }
    return success;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    bool success = false;
    if (_spa06_003_pressure == NULL || _spa06_003->isPresDataReady() == false) {
      return false;
    }
    success = _spa06_003_pressure->getEvent(pressureEvent);
    if (pressureEvent->pressure < SPA06_003_PRESSURE_MIN ||
        pressureEvent->pressure > SPA06_003_PRESSURE_MAX) {
      success = false;
    }
    return success;
  }

protected:
  Adafruit_SPA06_003 *_spa06_003 = nullptr; ///< SPA06-003 object
  Adafruit_Sensor *_spa06_003_temp = {
      0}; ///< Ptr to an adafruit_sensor representing the temperature
  Adafruit_Sensor *_spa06_003_pressure = {
      0}; ///< Ptr to an adafruit_sensor representing the pressure
};

#endif // WipperSnapper_I2C_Driver_SPA06_003_H