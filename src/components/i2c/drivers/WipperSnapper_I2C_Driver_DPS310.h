/*!
 * @file WipperSnapper_I2C_Driver_DPS310.h
 *
 * Device driver the DPS310 barometric pressure sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_DPS310_H
#define WipperSnapper_I2C_Driver_DPS310_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_DPS310.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the DPS310 barometric
            pressure sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_DPS310 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a DPS310 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_DPS310(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an DPS310 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_DPS310() { delete _dps310; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the DPS310 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    // initialize DPS310
    _dps310 = new Adafruit_DPS310();
    if (!_dps310->begin_I2C((uint8_t)_sensorAddress, _i2c))
      return false;

    // init OK, perform sensor configuration
    _dps310->configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    _dps310->configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    _dps_temp = _dps310->getTemperatureSensor();
    _dps_pressure = _dps310->getPressureSensor();
    // check if sensors are configured properly
    if (_dps_temp == NULL || _dps_pressure == NULL)
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the DPS310's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!_dps310->temperatureAvailable())
      return false;

    _dps_temp->getEvent(tempEvent);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the DPS310's pressure reading.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the pressure was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPressure(sensors_event_t *pressureEvent) {
    if (!_dps310->pressureAvailable())
      return false;

    _dps_pressure->getEvent(pressureEvent);
    return true;
  }

protected:
  Adafruit_DPS310 *_dps310; ///< DPS310 driver object
  Adafruit_Sensor *_dps_temp =
      NULL; ///< Holds data for the DPS310's temperature sensor
  Adafruit_Sensor *_dps_pressure =
      NULL; ///< Holds data for the DPS310's pressure sensor
};

#endif // WipperSnapper_I2C_Driver_DPS310