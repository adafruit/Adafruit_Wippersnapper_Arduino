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
      @brief    Checks if the sensor has a new measurement ready to read.
      @returns  True if a new measurement is ready, False otherwise.
  */
  bool IsSensorReady() {
    bool isDataReady = false;
    return (_sen->readDataReady(isDataReady) == 0) && isDataReady;
  }

  /*!
      @brief    Reads all SEN5X metrics in one transaction when new data is
                ready, leaving the cached members untouched otherwise so every
                metric in a read pass reflects the same sample. Serves the
                cached sample if the last read was under one second ago.
      @returns  True once a successful read has populated the cached values,
                False before the first successful read.
  */
  bool ReadSensorData() override {
    if (HasBeenReadInLastSecond())
      return _have_data;

    if (IsSensorReady()) {
      uint16_t error = _sen->readMeasuredValues(
          _massConcentrationPm1p0, _massConcentrationPm2p5,
          _massConcentrationPm4p0, _massConcentrationPm10p0, _ambientHumidity,
          _ambientTemperature, _vocIndex, _noxIndex);
      if (error == 0) {
        _last_read = millis();
        _have_data = true;
      }
    }
    return _have_data;
  }

  /*!
      @brief    Gets the SEN5X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadSensorData() || isnan(_ambientTemperature)) {
      return false;
    }

    tempEvent->temperature = _ambientTemperature;
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
    if (!ReadSensorData() || isnan(_ambientHumidity)) {
      return false;
    }

    humidEvent->relative_humidity = _ambientHumidity;
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
    if (!ReadSensorData() || isnan(_noxIndex)) {
      return false;
    }

    noxIndexEvent->nox_index = _noxIndex;
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
    if (!ReadSensorData() || isnan(_vocIndex)) {
      return false;
    }

    vocIndexEvent->voc_index = _vocIndex;
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
    if (!ReadSensorData() || isnan(_massConcentrationPm1p0) ||
        _massConcentrationPm1p0 == OVERFLOW_SEN55) {
      return false;
    }

    pm10StdEvent->pm10_std = _massConcentrationPm1p0;
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
    if (!ReadSensorData() || isnan(_massConcentrationPm2p5) ||
        _massConcentrationPm2p5 == OVERFLOW_SEN55) {
      return false;
    }

    pm25StdEvent->pm25_std = _massConcentrationPm2p5;
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
    if (!ReadSensorData() || isnan(_massConcentrationPm4p0) ||
        _massConcentrationPm4p0 == OVERFLOW_SEN55) {
      return false;
    }

    pm40StdEvent->data[0] = _massConcentrationPm4p0;
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
    if (!ReadSensorData() || isnan(_massConcentrationPm10p0) ||
        _massConcentrationPm10p0 == OVERFLOW_SEN55) {
      return false;
    }

    pm100StdEvent->pm100_std = _massConcentrationPm10p0;
    return true;
  }

protected:
  SensirionI2CSen5x *_sen = nullptr;    ///< SEN5X driver object
  float _massConcentrationPm1p0 = NAN;  ///< PM1.0 mass concentration
  float _massConcentrationPm2p5 = NAN;  ///< PM2.5 mass concentration
  float _massConcentrationPm4p0 = NAN;  ///< PM4.0 mass concentration
  float _massConcentrationPm10p0 = NAN; ///< PM10.0 mass concentration
  float _ambientHumidity = NAN;         ///< Ambient humidity
  float _ambientTemperature = NAN;      ///< Ambient temperature
  float _vocIndex = NAN;                ///< VOC index
  float _noxIndex = NAN;                ///< NOx index
};

#endif // drvSen5x
