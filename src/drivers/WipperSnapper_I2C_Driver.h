/*!
 * @file WipperSnapper_I2C_Driver.h
 *
 * Base implementation for I2C device drivers.
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

#ifndef WipperSnapper_I2C_Driver_H
#define WipperSnapper_I2C_Driver_H

#include "Wippersnapper.h"
#include <Adafruit_Sensor.h>

/** Types of I2C driver, corresponding to Driver_CLASSNAME.h */
typedef enum {
  UNSPECIFIED, // Unspecified/undefined i2c device driver.
  AHTX0,       // AHTX0 device driver
  DPS310,      // DPS310 device driver
  SCD30,       // SCD30 device driver
  SCD4X,       // SCD4X device driver
  PM25AQI,     // PM25AQI device driver
  BME280,      // BME280 device driver
  BME680       // BME680 device driver
} DriverType_t;

/**************************************************************************/
/*!
    @brief  Base class for I2C Drivers.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an I2C sensor.
      @param    _i2c
                Instance of the I2C object.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver(TwoWire *_i2c, uint16_t sensorAddress) {
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an I2C sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver() { _sensorAddress = 0; }

  void
  configureDriver(wippersnapper_i2c_v1_I2CDeviceInitRequest *msgDeviceInitReq) {
    int propertyIdx = 0;
    while (propertyIdx < msgDeviceInitReq->i2c_device_properties_count) {
      switch (
          msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_type) {
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_AMBIENT_TEMPERATURE:
        enableSensorAmbientTemperature();
        setSensorAmbientTemperaturePeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_RELATIVE_HUMIDITY:
        enableSensorRelativeHumidity();
        setSensorRelativeHumidityPeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_PRESSURE:
        enableSensorPressure();
        setSensorPressurePeriod(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
        break;
      case wippersnapper_i2c_v1_SensorType_SENSOR_TYPE_CO2:
        enableSensorCO2();
        setSensorCO2Period(
            msgDeviceInitReq->i2c_device_properties[propertyIdx].sensor_period);
      default:
        break;
      }
      ++propertyIdx;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the initialization status of an I2C driver.
      @returns  True if I2C device is initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool isInitialized() { return _isInitialized; }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C device's address.
      @returns  The I2C device's unique i2c address.
  */
  /*******************************************************************************/
  uint16_t sensorAddress() { return _sensorAddress; }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C device driver's type (corresponds to class name)
      @returns  The type of I2C driver in-use.
  */
  /*******************************************************************************/
  // DriverType_t getDriverType() { return driverType; }

  /*******************************************************************************/
  /*!
      @brief    Sets the I2C device driver's type.
      @param    type
                test
  */
  /*******************************************************************************/
  void setDriverType(DriverType_t type) { driverType = type; }

  /*******************************************************************************/
  /*!
      @brief    Enables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorAmbientTemperature(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorAmbientTemperature(){_tempSensorPeriod = 0.0};

  /*******************************************************************************/
  /*!
      @brief    Enables the device's relative humidity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorRelativeHumidity(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's relative humidity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorRelativeHumidity(){_humidSensorPeriod = 0.0};

  /*******************************************************************************/
  /*!
      @brief    Enables the device's pressure sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorPressure(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's pressure sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorPressure(){_pressureSensorPeriod = 0.0};

  /*******************************************************************************/
  /*!
      @brief    Enables the device's CO2 sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableSensorCO2(){};

  /*******************************************************************************/
  /*!
      @brief    Set the co2 sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
     co2 sensor.
  */
  /*******************************************************************************/
  virtual void setSensorCO2Period(float period) {
    if (period == 0) {
      disableSensorCO2();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _CO2SensorPeriod = (long)period * 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a CO2 sensor.
      @param    period
                The time interval at which to return new data from the CO2
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorCO2(float period) { setSensorCO2Period(period); }

  /*******************************************************************************/
  /*!
      @brief    Disables the device's CO2 sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableSensorCO2(){_CO2SensorPeriod = 0.0};

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the pressure sensor's period, if
     set.
      @returns  Time when the pressure sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorCO2Period() { return _CO2SensorPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pressure sensor was queried last.
      @returns  Time when the pressure sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorCO2PeriodPrv() { return _CO2SensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the pressure sensor was queried.
      @param    period
                The time when the pressure sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorCO2PeriodPrv(long period) {
    _CO2SensorPeriodPrv = period;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SCD4X's current humidity.
      @param    CO2Value
                The CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventCO2(float *CO2Value) { return false; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the humidity sensor's period, if
     set.
      @returns  Time when the temperature sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTemperaturePeriod() { return _tempSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the temperature sensor's return frequency.
      @param    period
                The time interval at which to return new data from the
     temperature sensor.
  */
  /*******************************************************************************/
  virtual void setSensorAmbientTemperaturePeriod(float period) {
    if (period == 0) {
      disableSensorAmbientTemperature();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _tempSensorPeriod = (long)period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the temperature sensor was queried last.
      @returns  Time when the temperature sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorAmbientTemperaturePeriodPrv() {
    return _tempSensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the temperature sensor was queried.
      @param    periodPrv
                The time when the temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorAmbientTemperaturePeriodPrv(long periodPrv) {
    _tempSensorPeriodPrv = periodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a temperature sensor. Expects value
                to return in the proper SI unit.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAmbientTemperature(sensors_event_t *tempEvent) {
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a temperature sensor. Expects value
                to return in the proper SI unit.
      @param    tempEvent
                Pointer to an temperature value.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventAmbientTemperature(float tempEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Update the properties of a temperature
                  sensor, provided sensor_period.
      @param    period
                Sensor's period.
  */
  /*******************************************************************************/
  virtual void updateSensorAmbientTemperature(float period) {
    setSensorAmbientTemperaturePeriod(period);
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the humidity sensor's period, if
     set.
      @returns  Time when the humidity sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorRelativeHumidityPeriod() { return _humidSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the humidity sensor's return frequency.
      @param    period
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  virtual void setSensorRelativeHumidityPeriod(float period) {
    if (period == 0) {
      disableSensorAmbientTemperature();
      return;
    }
    // Period is in seconds, cast it to long and convert it to milliseconds
    _humidSensorPeriod = (long)period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the humidity sensor was queried last.
      @returns  Time when the humidity sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorRelativeHumidityPeriodPrv() {
    return _humidSensorPeriodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the temperature sensor was queried.
      @param    periodPrv
                The time when the temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorRelativeHumidityPeriodPrv(long periodPrv) {
    _humidSensorPeriodPrv = periodPrv;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a humidity sensor and converts
                the reading into the expected SI unit.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a humidity sensor and converts
                the reading into the expected SI unit.
      @param    humidEvent
                Pointer to a humidity value.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventRelativeHumidity(float humidEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a relative humidity sensor.
      @param    period
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorRelativeHumidity(float period) {
    setSensorRelativeHumidityPeriod(period);
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the pressure sensor's period, if
     set.
      @returns  Time when the pressure sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorPressurePeriod() { return _pressureSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the pressure sensor's return frequency.
      @param    period
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  virtual void setSensorPressurePeriod(float period) {
    if (period == 0)
      disableSensorPressure();
    // Period is in seconds, cast it to long and convert it to milliseconds
    _pressureSensorPeriod = (long)period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pressure sensor was queried last.
      @returns  Time when the pressure sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long sensorPressurePeriodPrv() { return _pressureSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the pressure sensor was queried.
      @param    period
                The time when the pressure sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setSensorPressurePeriodPrv(long period) {
    _pressureSensorPeriodPrv = period;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPressure(sensors_event_t *pressureEvent) {
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a pressure sensor and converts
                the reading into the expected SI unit.
      @param    pressureEvent
                A pressure value
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventPressure(float pressureEvent) { return false; }

  /*******************************************************************************/
  /*!
      @brief    Updates the properties of a pressure sensor.
      @param    period
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  virtual void updateSensorPressure(float period) {
    setSensorPressurePeriod(period);
  }

  DriverType_t driverType = UNSPECIFIED; ///< The type of I2C driver.
protected:
  bool _isInitialized = false; ///< True if the I2C device was initialized
                               ///< successfully, False otherwise.
  uint16_t _sensorAddress;     ///< The I2C device's unique I2C address.
  long _tempSensorPeriod =
      0L; ///< The time period between reading the temperature sensor's value.
  long _tempSensorPeriodPrv =
      0L; ///< The time when the temperature sensor was last read
  long _humidSensorPeriod =
      0L; ///< The time period between reading the humidity sensor's value.
  long _humidSensorPeriodPrv = 0L; ///< The time when the humidity sensor was
                                   ///< last read.
  long _pressureSensorPeriod =
      0L; ///< The time period between reading the pressure sensor's value.
  long _pressureSensorPeriodPrv = 0L; ///< The time when the pressure sensor
                                      ///< was last read.
  long _CO2SensorPeriod =
      0L; ///< The time period between reading the CO2 sensor's value.
  long _CO2SensorPeriodPrv = 0L; ///< The time when the CO2 sensor
                                 ///< was last read.
};

#endif // WipperSnapper_I2C_Driver_H