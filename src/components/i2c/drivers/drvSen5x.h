/*!
 * @file drvSen5x.h
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

#ifndef DRV_SEN5X_H
#define DRV_SEN5X_H

#include "drvBase.h"
#include <SensirionI2CSen5x.h>
#include <Wire.h>

/*!
    @brief  Class that provides a driver interface for the SEN5X sensor.
*/
class drvSen5x : public drvBase {

  const float OVERFLOW_SEN55 = (0xFFFF / 10); // maxes out at u_int16 / 10

public:
  /*!
      @brief    Constructor for a SEN5X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvSen5x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the SEN5X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
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

  /*!
      @brief    Gets the SEN5X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || ambientTemperature == NAN) {
      return false;
    }

    tempEvent->temperature = ambientTemperature;
    return true;
  }

  /*!
      @brief    Gets the SEN5X's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || ambientHumidity == NAN) {
      return false;
    }

    humidEvent->relative_humidity = ambientHumidity;
    return true;
  }

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
  bool getEventNOxIndex(sensors_event_t *noxIndexEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || noxIndex == NAN) {
      return false;
    }

    noxIndexEvent->nox_index = noxIndex;
    return true;
  }

  /*!
      @brief    Gets the SEN5X's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || vocIndex == NAN) {
      return false;
    }

    vocIndexEvent->voc_index = vocIndex;
    return true;
  }

  /*!
      @brief    Gets the SEN5X sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || massConcentrationPm1p0 == NAN ||
        massConcentrationPm1p0 == OVERFLOW_SEN55) {
      return false;
    }

    pm10StdEvent->pm10_std = massConcentrationPm1p0;
    return true;
  }

  /*!
      @brief    Gets the SEN5X sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || massConcentrationPm2p5 == NAN ||
        massConcentrationPm2p5 == OVERFLOW_SEN55) {
      return false;
    }

    pm25StdEvent->pm25_std = massConcentrationPm2p5;
    return true;
  }

  /*!
      @brief    Gets the SEN5X sensor's PM4.0 STD reading.
      @param    pm40StdEvent
                  Adafruit Sensor event for PM4.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM40_STD(sensors_event_t *pm40StdEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || massConcentrationPm4p0 == NAN ||
        massConcentrationPm4p0 == OVERFLOW_SEN55) {
      return false;
    }

    pm40StdEvent->data[0] = massConcentrationPm4p0;
    return true;
  }

  /*!
      @brief    Gets the SEN5X sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    float massConcentrationPm1p0, massConcentrationPm2p5,
        massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity,
        ambientTemperature, vocIndex, noxIndex;
    uint16_t error;

    error = _sen->readMeasuredValues(
        massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
        massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
        noxIndex);
    if (error != 0 || massConcentrationPm10p0 == NAN ||
        massConcentrationPm10p0 == OVERFLOW_SEN55) {
      return false;
    }

    pm100StdEvent->pm100_std = massConcentrationPm10p0;
    return true;
  }

protected:
  SensirionI2CSen5x *_sen; ///< SEN5X driver object
};

#endif // drvSen5x
