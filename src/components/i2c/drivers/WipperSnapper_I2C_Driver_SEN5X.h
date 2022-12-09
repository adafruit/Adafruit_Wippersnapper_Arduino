/*!
 * @file WipperSnapper_I2C_Driver_SEN5X.h
 *
 * Device driver for the SEN5X CO2, Temperature, and Humidity sensor.
 * TEMPORARY HACK
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Marni Brewster 2022 for Adafruit Industries.
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
class WipperSnapper_I2C_Driver_SEN5X : public WipperSnapper_I2C_Driver
{

  const float OVERFLOW_SEN55 = (0xFFFF / 10);  // maxes out at u_int16 / 10

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
      : WipperSnapper_I2C_Driver(i2c, sensorAddress)
  {
    _i2c = i2c;
    _sensorAddress = sensorAddress;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SEN5X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin()
  {
    _sen = new SensirionI2CSen5x();
    _sen->begin(*_i2c);
    u_int16_t error_stop = _sen->deviceReset();
    if (error_stop != 0)
    {
      return false;
    }
    u_int16_t error_start = _sen->startMeasurement();
    if (error_start != 0)
    {
      return false;
    }

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
  bool getEventAmbientTemp(sensors_event_t *tempEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_tempSensorPeriod != 0 && error != 0) || ambientTemperature == NAN)
    {
      return false;
    }

    tempEvent->temperature = ambientTemperature;
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
  bool getEventRelativeHumidity(sensors_event_t *humidEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_humidSensorPeriod != 0 && error != 0) || ambientHumidity == NAN)
    {
      return false;
    }

    humidEvent->relative_humidity = ambientHumidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN5X's current NOX/VOC reading.
      @param    rawEvent
                  Adafruit Sensor event for Raw data (4 float array)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_rawSensorPeriod != 0 && error) || noxIndex == 0 || vocIndex == 0)
    {
      return false;
    }

    rawEvent->data[0] = noxIndex; // alphabetical?
    rawEvent->data[1] = vocIndex;
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
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_PM10SensorPeriod != 0 && error != 0) || massConcentrationPm1p0 == NAN ||
        massConcentrationPm1p0 == OVERFLOW_SEN55)
    {
      return false;
    }

    pm10StdEvent->data[0] = massConcentrationPm1p0;
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
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_PM25SensorPeriod != 0 && error != 0) || massConcentrationPm2p5 == NAN ||
        massConcentrationPm2p5 == OVERFLOW_SEN55)
    {
      return false;
    }

    pm25StdEvent->data[0] = massConcentrationPm10p0;
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
  bool getEventPM40_STD(sensors_event_t *pm40StdEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_PM25SensorPeriod != 0 && error != 0) || massConcentrationPm4p0 == NAN ||
        massConcentrationPm4p0 == OVERFLOW_SEN55)
    {
      return false;
    }

    pm40StdEvent->data[0] = massConcentrationPm4p0;
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
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent)
  {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0,
        ambientHumidity, ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5,
                                     massConcentrationPm4p0, massConcentrationPm10p0,
                                     ambientHumidity, ambientTemperature, vocIndex, noxIndex);
    if ((_PM100SensorPeriod != 0 && error != 0) || massConcentrationPm10p0 == NAN ||
        massConcentrationPm10p0 == OVERFLOW_SEN55)
    {
      return false;
    }

    pm100StdEvent->data[0] = massConcentrationPm10p0;
    return true;
  }

protected:
  SensirionI2CSen5x *_sen; ///< SEN5X driver object
};

#endif // WipperSnapper_I2C_Driver_SEN5X