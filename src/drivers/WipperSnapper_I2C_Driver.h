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
  PM25AQI      // PM25AQI device driver
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

  /*******************************************************************************/
  /*!
      @brief    Gets the initialization status of an I2C driver.
      @returns  True if I2C device is initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool getInitialized() { return _isInitialized; }

  /*******************************************************************************/
  /*!
      @brief    Gets the I2C device's address.
      @returns  The I2C device's unique i2c address.
  */
  /*******************************************************************************/
  uint16_t getSensorAddress() { return _sensorAddress; }

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
                The type of I2C driver (corresponds to header Driver_.h class
     name)
  */
  /*******************************************************************************/
  void setDriverType(DriverType_t type) { driverType = type; }

  /*******************************************************************************/
  /*!
      @brief    Enables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableTemperatureSensor(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableTemperatureSensor(){};

  /*******************************************************************************/
  /*!
      @brief    Enables the device's humidity sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableHumiditySensor(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's temperature sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableHumiditySensor(){};

  /*******************************************************************************/
  /*!
      @brief    Enables the device's pressure sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enablePressureSensor(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's pressure sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disablePressureSensor(){};

  /*******************************************************************************/
  /*!
      @brief    Enables the device's CO2 sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void enableCO2Sensor(){};

  /*******************************************************************************/
  /*!
      @brief    Disables the device's CO2 sensor, if it exists.
  */
  /*******************************************************************************/
  virtual void disableCO2Sensor(){};

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the humidity sensor's period, if
     set.
      @returns  Time when the temperature sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getTempSensorPeriod() { return _tempSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the temperature sensor's return frequency.
      @param    tempPeriod
                The time interval at which to return new data from the
     temperature sensor.
  */
  /*******************************************************************************/
  virtual void setTemperatureSensorPeriod(float tempPeriod) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _tempSensorPeriod = (long)tempPeriod * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the temperature sensor was queried last.
      @returns  Time when the temperature sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getTempSensorPeriodPrv() { return _tempSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the temperature sensor was queried.
      @param    tempPeriodPrv
                The time when the temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setTemperatureSensorPeriodPrv(long tempPeriodPrv) {
    _tempSensorPeriodPrv = tempPeriodPrv;
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
  virtual bool getTemp(sensors_event_t *tempEvent) { return true; }

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
  virtual bool getTemp(float *tempEvent) { return true; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the humidity sensor's period, if
     set.
      @returns  Time when the humidity sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getHumidSensorPeriod() { return _humidSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the humidity sensor's return frequency.
      @param    humidPeriod
                The time interval at which to return new data from the humidity
                sensor.
  */
  /*******************************************************************************/
  void setHumiditySensorPeriod(float humidPeriod) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _humidSensorPeriod = (long)humidPeriod * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the humidity sensor was queried last.
      @returns  Time when the humidity sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getHumidSensorPeriodPrv() { return _humidSensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Sets a timestamp for when the temperature sensor was queried.
      @param    humidPeriodPrv
                The time when the temperature sensor was queried last.
  */
  /*******************************************************************************/
  virtual void setHumidSensorPeriodPrv(long humidPeriodPrv) {
    _humidSensorPeriodPrv = humidPeriodPrv;
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
  virtual bool getHumid(sensors_event_t *humidEvent) { return true; }

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
  virtual bool getHumid(float *humidEvent) { return true; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the pressure sensor's period, if
     set.
      @returns  Time when the pressure sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getPressureSensorPeriod() { return _pressureSensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the pressure sensor's return frequency.
      @param    pressurePeriod
                The time interval at which to return new data from the pressure
                sensor.
  */
  /*******************************************************************************/
  void setPressureSensorPeriod(float pressurePeriod) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _pressureSensorPeriod = (long)pressurePeriod * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
                    which the pressure sensor was queried last.
      @returns  Time when the pressure sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getPressureSensorPeriodPrv() { return _pressureSensorPeriodPrv; }

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
  virtual bool getPressure(sensors_event_t *pressureEvent) { return true; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the gas sensor's period, if
     set.
      @returns  Time when the CO2 sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getCO2SensorPeriod() { return _CO2SensorPeriod; }

  /*******************************************************************************/
  /*!
      @brief    Set the CO2 sensor's return frequency.
      @param    CO2Period
                The time interval at which to return new data from the CO2
                sensor.
  */
  /*******************************************************************************/
  void setCO2SensorPeriod(float CO2Period) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _CO2SensorPeriod = (long)CO2Period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the CO2 sensor was queried last.
      @returns  Time when the CO2 sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getCO2SensorPeriodPrv() { return _CO2SensorPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a CO2 sensor.
      @param    CO2Value
                    The CO2 value, in ppm.
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getCO2(float *CO2Value) { return true; }

  /*******************************************************************************/
  /*!
      @brief    Set the PM10 sensor's return frequency.
      @param    pm10Period
                The time interval at which to return new data from the PM
                sensor.
  */
  /*******************************************************************************/
  void setPM10STDPeriod(float pm10Period) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _pm10STDPeriod = (long)pm10Period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the PM sensor's period, if
     set.
      @returns  Time when the PM sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getPM10STDSensorPeriod() { return _pm10STDPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the PM sensor was queried last.
      @returns  Time when the PM sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getPM10STDSensorPeriodPrv() { return _pm10STDPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Set the 25 sensor's return frequency.
      @param    pm25Period
                The time interval at which to return new data from the PM
                sensor.
  */
  /*******************************************************************************/
  void setPM25STDPeriod(float pm25Period) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _pm25STDPeriod = (long)pm25Period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the PM sensor's period, if
     set.
      @returns  Time when the PM sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getPM25STDSensorPeriod() { return _pm25STDPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the PM sensor was queried last.
      @returns  Time when the PM sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getPM25STDSensorPeriodPrv() { return _pm25STDPeriodPrv; }

  /*******************************************************************************/
  /*!
      @brief    Set the PM100 sensor's return frequency.
      @param    pm100
                The time interval at which to return new data from the PM
                sensor.
  */
  /*******************************************************************************/
  void setPM100STDPeriod(float pm100Period) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _pm100STDPeriod = (long)pm100Period * 1000;
  }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the PM sensor's period, if
     set.
      @returns  Time when the PM sensor should be polled, in seconds.
  */
  /*********************************************************************************/
  virtual long getPM100STDSensorPeriod() { return _pm100STDPeriod; }

  /*********************************************************************************/
  /*!
      @brief    Base implementation - Returns the previous time interval at
     which the PM sensor was queried last.
      @returns  Time when the PM sensor was last queried, in seconds.
  */
  /*********************************************************************************/
  virtual long getPM100STDSensorPeriodPrv() { return _pm100STDPeriodPrv; }

  DriverType_t driverType = UNSPECIFIED; ///< The type of I2C driver.
protected:
  bool _isInitialized = false; ///< True if the I2C device was initialized
                               ///< successfully, False otherwise.
  uint16_t _sensorAddress;     ///< The I2C device's unique I2C address.
  long _tempSensorPeriod =
      -1L; ///< The time period between reading the temperature sensor's value.
  long _tempSensorPeriodPrv =
      -1L; ///< The time when the temperature sensor was last read
  long _humidSensorPeriod =
      -1L; ///< The time period between reading the humidity sensor's value.
  long _humidSensorPeriodPrv = -1L; ///< The time when the humidity sensor was
                                    ///< last read.
  long _pressureSensorPeriod =
      -1L; ///< The time period between reading the pressure sensor's value.
  long _pressureSensorPeriodPrv = -1L; ///< The time when the pressure sensor
                                       ///< was last read.
  long _CO2SensorPeriod =
      -1L; ///< The time period between reading the CO2 sensor's value.
  long _CO2SensorPeriodPrv; ///< The time when the CO2 sensor
                            ///< was last read.
  long _pm10STDPeriod = -1L;
  long _pm10STDPeriodPrv = -1L;
  long _pm25STDPeriod = -1L;
  long _pm25STDPeriodPrv = -1L;
  long _pm100STDPeriod = -1L;
  long _pm100STDPeriodPrv = -1L;
};

#endif // WipperSnapper_I2C_Driver_H