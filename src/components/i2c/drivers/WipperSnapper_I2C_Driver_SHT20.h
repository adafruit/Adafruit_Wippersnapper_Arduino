/*!
 * @file WipperSnapper_I2C_Driver_SHT20.h
 *
 * Device driver for the SHT20 Temperature and Humidity Sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Adafruit Wippersnapper code Copyright (c) 2022 Adafruit Industries.
 * SHT20 Driver code Copyright (c) DFRobot 2022. 
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_SHT20_H
#define WipperSnapper_I2C_Driver_SHT20_H

#include "WipperSnapper_I2C_Driver.h"
#include <DFRobot_SHT20.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SHT20 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SHT20 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SHT20 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SHT20(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SHT20 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

    _SHT20 = new DFRobot_SHT20(_i2c, _sensorAddress);
    delay(100);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHT20's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    // populate temp and humidity objects with fresh data
    tempEvent->temperature = _SHT20->readTemperature();
    // the driver returns a sketchy float if an error occurs, this could be caught
    // here with a sensible range check and return false if not.
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SHT20's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    // populate temp and humidity objects with fresh data
    humidEvent->relative_humidity = _SHT20->readHumidity();
    // the driver returns a sketchy float if an error occurs, this could be caught
    // here with a sensible range check and return false if not.
    return true;
  }

protected:
  DFRobot_SHT20 *_SHT20; ///< SHT20 object
};

#endif // WipperSnapper_I2C_Driver_SHT20