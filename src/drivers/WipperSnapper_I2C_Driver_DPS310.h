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
      @param    _i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_DPS310(TwoWire *_i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(_i2c, sensorAddress) {
    setDriverType(DPS310);
    _sensorAddress = sensorAddress;
    _isInitialized = _dps310.begin_I2C((uint8_t)_sensorAddress, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an DPS310 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_DPS310() {
    _dps_temp = NULL;
    _tempSensorPeriod = 0L;
    _dps_pressure = NULL;
    _pressureSensorPeriod = 0L;
    setDriverType(UNSPECIFIED);
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the DPS310's temperature sensor. Sets highest precision
                options
  */
  /*******************************************************************************/
  void enableSensorAmbientTemperature() {
    _dps310.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
    _dps_temp = _dps310.getTemperatureSensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Enables the DPS310's pressure sensor.
  */
  /*******************************************************************************/
  void enableSensorPressure() {
    _dps310.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    _dps_pressure = _dps310.getPressureSensor();
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the DPS310's pressure sensor.
  */
  /*******************************************************************************/
  void disableSensorAmbientTemperature() {
    _dps_temp = NULL;
    _tempSensorPeriod = 0L;
  }

  /*******************************************************************************/
  /*!
      @brief    Disables the DPS310's pressure sensor.
  */
  /*******************************************************************************/
  void disableSensorPressure() {
    _dps_pressure = NULL;
    _pressureSensorPeriod = 0L;
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of an ambient temperature
                  sensor, provided sensor_period.
      @param    period
                Sensor's period.
  */
  /*******************************************************************************/
  void updateSensorAmbientTemperature(float period) {
    // disable the sensor
    if (period == 0)
      disableSensorAmbientTemperature();
    // enable a previously disabled sensor
    if (period > 0 && _dps_temp == NULL)
      enableSensorAmbientTemperature();

    setSensorAmbientTemperaturePeriod(period);
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a pressure sensor.
      @param    period
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  void updateSensorPressure(float period) {
    // disable the sensor
    if (period == 0)
      disableSensorPressure();
    // enable a previously disabled sensor
    if (period > 0 && _dps_pressure == NULL)
      enableSensorPressure();

    setSensorPressurePeriod(period);
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
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    if (_dps_temp != NULL && _dps310.temperatureAvailable()) {
      _dps_temp->getEvent(tempEvent);
      return true;
    }
    return false;
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
    if (_dps_pressure != NULL && _dps310.pressureAvailable()) {
      _dps_pressure->getEvent(pressureEvent);
      return true;
    }
    return false;
  }

protected:
  Adafruit_DPS310 _dps310; ///< DPS310 driver object
  Adafruit_Sensor *_dps_temp =
      NULL; ///< Holds data for the DPS310's temperature sensor
  Adafruit_Sensor *_dps_pressure =
      NULL; ///< Holds data for the DPS310's pressure sensor
};

#endif // WipperSnapper_I2C_Driver_DPS310