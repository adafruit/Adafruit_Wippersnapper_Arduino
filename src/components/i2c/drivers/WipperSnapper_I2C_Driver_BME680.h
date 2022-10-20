/*!
 * @file WipperSnapper_I2C_Driver_BME680.h
 *
 * Device driver for an AHT Humidity and Temperature sensor.
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

#ifndef WipperSnapper_I2C_Driver_BME680_H
#define WipperSnapper_I2C_Driver_BME680_H

#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_BME680.h>

#define SEALEVELPRESSURE_HPA (1013.25) ///< Default sea level pressure, in hPa

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for the BME680 temperature
            and humidity sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_BME680 : public WipperSnapper_I2C_Driver {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BME680 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_BME680(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an BME680 sensor.
  */
  /*******************************************************************************/
  ~WipperSnapper_I2C_Driver_BME680() { delete _bme; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the BME680 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _bme = new Adafruit_BME680(_i2c);

    // attempt to initialize BME680
    if (!_bme->begin())
      return false;

    // Set up oversampling and filter initialization
    // defaults from
    // https://github.com/adafruit/Adafruit_BME680/blob/master/examples/bme680test/bme680test.ino
    _bme->setTemperatureOversampling(BME680_OS_8X);
    _bme->setHumidityOversampling(BME680_OS_2X);
    _bme->setPressureOversampling(BME680_OS_4X);
    _bme->setIIRFilterSize(BME680_FILTER_SIZE_3);
    _bme->setGasHeater(320, 150); // 320*C for 150 ms

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a reading in blocking mode.
      @returns  True if the reading succeeded, False otherwise.
  */
  /*******************************************************************************/
  bool bmePerformReading() { return _bme->performReading(); }

  /*******************************************************************************/
  /*!
      @brief    Gets the BME680's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!bmePerformReading())
      return false;
    tempEvent->temperature = _bme->temperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the BME680's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (!bmePerformReading())
      return false;
    humidEvent->relative_humidity = _bme->humidity;
    return true;
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
    if (!bmePerformReading())
      return false;
    pressureEvent->pressure = (float)_bme->pressure;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads a the BME680's altitude sensor into an event.
      @param    altitudeEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAltitude(sensors_event_t *altitudeEvent) {
    if (!bmePerformReading())
      return false;
    // NOTE: This is hacked onto Adafruit_Sensor and should eventually be
    // removed
    altitudeEvent->data[0] = (float)_bme->readAltitude(SEALEVELPRESSURE_HPA);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Base implementation - Reads a gas resistance sensor and converts
                the reading into the expected SI unit.
      @param    gasEvent
                gas resistance sensor reading, in ohms.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  virtual bool getEventGasResistance(sensors_event_t *gasEvent) {
    if (!bmePerformReading())
      return false;
    // NOTE: This is hacked onto Adafruit_Sensor and should eventually be
    // removed
    gasEvent->data[0] = (float)_bme->gas_resistance;
    return true;
  }

protected:
  Adafruit_BME680 *_bme; ///< BME680 object
};

#endif // WipperSnapper_I2C_Driver_BME680