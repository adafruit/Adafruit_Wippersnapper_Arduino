/*!
 * @file WipperSnapper_I2C_Driver_SEN5X.h
 *
 * Device driver for the SEN5X air quality sensors.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_SEN5X_H
#define WipperSnapper_I2C_Driver_SEN5X_H

#include "WipperSnapper_I2C_Driver.h"
#include <SensirionI2CSen5x.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SEN5X sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_SEN5X : public WipperSnapper_I2C_Driver {

  const float OVERFLOW_SEN55 = (0xFFFF / 10); // maxes out at u_int16 / 10

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SEN5X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
  */
  /*******************************************************************************/
  WipperSnapper_I2C_Driver_SEN5X(TwoWire *i2c, uint16_t sensorAddress)
      : WipperSnapper_I2C_Driver(i2c, sensorAddress) {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
    _massConcentrationPm1p0 = NAN;
    _massConcentrationPm2p5 = NAN;
    _massConcentrationPm4p0 = NAN;
    _massConcentrationPm10p0 = NAN;
    _ambientHumidity = NAN;
    _ambientTemperature = NAN;
    _vocIndex = NAN;
    _noxIndex = NAN;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SEN5X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _sen = new SensirionI2CSen5x();
    _sen->begin(*_i2c);
    u_int16_t error_stop = _sen->deviceReset();
    if (error_stop != 0) {
      return false;
    }
    // Wait 1 second for sensors to start recording + 100ms for reset command
    delay(1100);
    u_int16_t error_start = _sen->startMeasurement();
    if (error_start != 0) {
      return false;
    }

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if sensor was read within last 1s, or is the first read.
      @returns  True if the sensor was recently read, False otherwise.
  */
  bool HasBeenReadInLastSecond() {
    return _lastRead != 0 && millis() - _lastRead < 1000;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if the sensor is ready to be read
      @returns  True if the sensor is ready, False otherwise.
  */
  /*******************************************************************************/
  bool IsSensorReady() {
    bool isDataReady = false;
    for (int i = 0; i < 2; i++) {
      uint16_t error = _sen->readDataReady(isDataReady);
      if (error == 0 && isDataReady) {
        return true;
      }
      delay(100);
    }
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() {
    // dont read sensor more than once per second
    if (HasBeenReadInLastSecond()) {
      return true;
    }

    if (!IsSensorReady()) {
      return false;
    }

    uint16_t error = _sen->readMeasuredValues(
        _massConcentrationPm1p0, _massConcentrationPm2p5,
        _massConcentrationPm4p0, _massConcentrationPm10p0, _ambientHumidity,
        _ambientTemperature, _vocIndex, _noxIndex);
    if (error != 0) {
      return false;
    }
    _lastRead = millis();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadSensorData() || _ambientTemperature == NAN) {
      return false;
    }
    tempEvent->temperature = _ambientTemperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (!ReadSensorData() || _ambientHumidity == NAN) {
      return false;
    }
    humidEvent->relative_humidity = _ambientHumidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X's current NOX reading.
                Note: If this value is unknown, which is true for SEN54,
                NAN is returned. During the first 10..11 seconds after
                power-on or device reset, this value will be NAN as well.
      @param    noxIndexEvent
                  Adafruit Sensor event for NOx Index (0-500, 1 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventNOxIndex(sensors_event_t *noxIndexEvent) {
    if (!ReadSensorData() || _noxIndex == NAN) {
      return false;
    }
    noxIndexEvent->nox_index = _noxIndex;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    if (!ReadSensorData() || _vocIndex == NAN) {
      return false;
    }
    vocIndexEvent->voc_index = _vocIndex;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    if (!ReadSensorData() || _massConcentrationPm1p0 == NAN ||
        _massConcentrationPm1p0 == OVERFLOW_SEN55) {
      return false;
    }
    pm10StdEvent->pm10_std = _massConcentrationPm1p0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    if (!ReadSensorData() || _massConcentrationPm2p5 == NAN ||
        _massConcentrationPm2p5 == OVERFLOW_SEN55) {
      return false;
    }
    pm25StdEvent->pm25_std = _massConcentrationPm2p5;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X sensor's PM4.0 STD reading.
      @param    pm40StdEvent
                  Adafruit Sensor event for PM4.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM40_STD(sensors_event_t *pm40StdEvent) {
    if (!ReadSensorData() || _massConcentrationPm4p0 == NAN ||
        _massConcentrationPm4p0 == OVERFLOW_SEN55) {
      return false;
    }
    pm40StdEvent->data[0] = _massConcentrationPm4p0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    if (!ReadSensorData() || _massConcentrationPm10p0 == NAN ||
        _massConcentrationPm10p0 == OVERFLOW_SEN55) {
      return false;
    }
    pm100StdEvent->pm100_std = _massConcentrationPm10p0;
    return true;
  }

protected:
  SensirionI2CSen5x *_sen = nullptr; ///< SEN5X driver object
  float _massConcentrationPm1p0;     ///< PM1.0 mass concentration
  float _massConcentrationPm2p5;     ///< PM2.5 mass concentration
  float _massConcentrationPm4p0;     ///< PM4.0 mass concentration
  float _massConcentrationPm10p0;    ///< PM10.0 mass concentration
  float _ambientHumidity;            ///< Ambient humidity
  float _ambientTemperature;         ///< Ambient temperature
  float _vocIndex;                   ///< VOC index
  float _noxIndex;                   ///< NOx index
  ulong _lastRead = 0uL;             ///< Last time the sensor was read
};

#endif // WipperSnapper_I2C_Driver_SEN5X
