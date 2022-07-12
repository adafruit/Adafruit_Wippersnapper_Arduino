/*!
 * @file WipperSnapper_I2C_Driver_SCD40.h
 *
 * Device driver for the SCD40 CO2, Temperature, and Humidity sensor.
 * TEMPORARY HACK
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_SCD40_H
#define WipperSnapper_I2C_Driver_SCD40_H

#include "WipperSnapper_I2C_Driver.h"
#include <SensirionI2CScd4x.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SCD40 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SCD40 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SCD40 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SCD40(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SCD40 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _scd = new SensirionI2CScd4x();  
    _scd->begin(*_i2c);
    bool error = _scd->stopPeriodicMeasurement();
    if (error) {
      return false;
    } 

    bool error_1 = _scd->startPeriodicMeasurement();          
    if (error_1) {
      return false;
    }
      
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD40's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    // check if sensor is enabled and data is available      
    _error = _scd->readMeasurement(_co2, _temperature, _humidity);
    if (_tempSensorPeriod != 0 && _error)
      return false;

    tempEvent->temperature = _temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD40's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // check if sensor is enabled and data is available   
    if (_humidSensorPeriod != 0 && _error)
      return false;
    
    if (_humidity) {
      // This sensor library does not implement Adafruit_Sensor       
      humidEvent->relative_humidity = _humidity;      
    } else {
      humidEvent->relative_humidity = 0;      
    }
    
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD40's current CO2 reading.
      @param    co2Event
                  Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    // check if sensor is enabled and data is available   
    if (_CO2SensorPeriod != 0 && _error)
      return false;       
    
    if (_co2) {
      co2Event->data[0] = _co2;
    } else {
      co2Event->data[0] = 0;
    }
    return true;
  }

protected:
  SensirionI2CScd4x *_scd; ///< SCD40 driver object
  uint16_t _co2;
  float _temperature;
  float _humidity;
  uint16_t _error;
};

#endif // WipperSnapper_I2C_Driver_SCD40