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

/** Types of I2C driver, corresponding to Driver_CLASSNAME.h */
typedef enum DriverType {
  UNSPECIFIED, // Unspecified/undefined i2c device driver.
  AHTX0        // AHTX0 Driver
} DriverType;

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
  static DriverType getDriverType() { return _driverType; }

  /*******************************************************************************/
  /*!
      @brief    Sets the I2C device driver's type.
      @param    driverType
                The type of I2C driver (corresponds to header Driver_.h class
     name)
  */
  /*******************************************************************************/
  void setDriverType(DriverType driverType) { _driverType = driverType; }

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
  virtual void setTemperatureSensorPeriodPrv(float tempPeriodPrv) {
    // Period is in seconds, cast it to long and convert it to milliseconds
    _tempSensorPeriodPrv = (long)tempPeriodPrv * 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a temperature sensor. Expects value
                to return in the proper SI unit.
      @param    temperature
                Pointer to a temperature value.
  */
  /*******************************************************************************/
  virtual void updateTempSensor(float *temperature) {
    // no-op
  }

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
  virtual long getHumidSensorPeriodPrv() { return _humidSensorPeriodPrv;}

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a humidity sensor and converts
                the reading into the expected SI unit.
      @param    humidity
                Pointer to a humidity value.
  */
  /*******************************************************************************/
  virtual void updateHumidSensor(float *humidity) {
    // no-op
  }

protected:
  bool _isInitialized = false; ///< True if the I2C device was initialized
                               ///< successfully, False otherwise.
  uint16_t _sensorAddress;     ///< The I2C device's unique I2C address.
  DriverType _driverType; ///< The type of I2C driver.
  long _tempSensorPeriod =
      -1L; ///< The time period between reading the temperature sensor's value.
  long _humidSensorPeriod =
      -1L; ///< The time period between reading the humidity sensor's value.
  long _tempSensorPeriodPrv;  ///< The time period when the temperature sensor
                              ///< was last read.
  long _humidSensorPeriodPrv; ///< The time period when the humidity sensor was
                              ///< last read.
};

#endif // WipperSnapper_I2C_Driver_H