/*!
 * @file drvSen6x.h
 *
 * Device driver for the SEN66 Particulate Matter, Temperature, Humidity, VOC,
 * NOX, and CO2 sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 * Modified (c) by Martin Ebner 2024 https://github.com/MartinEbnerSensirion
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_SEN6X_H
#define DRV_SEN6X_H

#include "drvBase.h"
#include <SensirionI2cSen66.h>
#include <Wire.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SEN6X sensor.
*/
/**************************************************************************/
class drvSen6x : public drvBase {

  const float OVERFLOW_SEN6X = (0xFFFF / 10); // maxes out at u_int16 / 10

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a SEN6X sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvSen6x(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _massConcentrationPm1p0 = NAN;
    _massConcentrationPm2p5 = NAN;
    _massConcentrationPm4p0 = NAN;
    _massConcentrationPm10p0 = NAN;
    _ambientHumidity = NAN;
    _ambientTemperature = NAN;
    _vocIndex = NAN;
    _noxIndex = NAN;
    _co2 = 0uL;
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the SEN6X sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() {
    _sen = new SensirionI2cSen66();
    _sen->begin(*_i2c, (uint8_t)_address);
    uint16_t error_stop = _sen->deviceReset();
    if (error_stop != 0) {
      return false;
    }
    // Wait 1 second for sensors to start recording + 100ms for reset command
    delay(1100);
    uint16_t error_start = _sen->startContinuousMeasurement();
    if (error_start != 0) {
      return false;
    }

    return true;

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Temperature offset / acceleration parameters (setTemperatureOffset...)
    //    to compensate for self-heating in an enclosure.
    //  - Ambient pressure / altitude for CO2 compensation
    //    (setAmbientPressure / setSensorAltitude).
    //  - VOC/NOx algorithm tuning parameters.
    //  - Automatic fan cleaning interval.
  }

  /*******************************************************************************/
  /*!
      @brief    Checks if the sensor has a new measurement ready.
      @returns  True if a new measurement is ready, False otherwise.
  */
  /*******************************************************************************/
  bool IsSensorReady() {
    bool isDataReady = false;
    uint8_t padding = 0x0;
    return (_sen->getDataReady(padding, isDataReady) == 0) && isDataReady;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads all SEN6X metrics in one transaction when new data is
                ready, leaving the cached members untouched otherwise so every
                metric in a read pass reflects the same sample. Serves the
                cached sample if the last read was under one second ago.
      @returns  True once a successful read has populated the cached values,
                False before the first successful read.
  */
  /*******************************************************************************/
  bool ReadSensorData() override {
    if (HasBeenReadInLastSecond())
      return _have_data;

    if (IsSensorReady()) {
      uint16_t error = _sen->readMeasuredValues(
          _massConcentrationPm1p0, _massConcentrationPm2p5,
          _massConcentrationPm4p0, _massConcentrationPm10p0, _ambientHumidity,
          _ambientTemperature, _vocIndex, _noxIndex, _co2);
      if (error == 0) {
        _last_read = millis();
        _have_data = true;
      }
    }
    return _have_data;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadSensorData() || isnan(_ambientTemperature)) {
      return false;
    }
    tempEvent->temperature = _ambientTemperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (!ReadSensorData() || isnan(_ambientHumidity)) {
      return false;
    }
    humidEvent->relative_humidity = _ambientHumidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X's current NOX reading.
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
    if (!ReadSensorData() || isnan(_noxIndex)) {
      return false;
    }
    noxIndexEvent->nox_index = _noxIndex;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X's current VOC reading.
      @param    vocIndexEvent
                  Adafruit Sensor event for VOC Index (1-500, 100 is normal)
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventVOCIndex(sensors_event_t *vocIndexEvent) {
    if (!ReadSensorData() || isnan(_vocIndex)) {
      return false;
    }
    vocIndexEvent->voc_index = _vocIndex;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    if (!ReadSensorData() || isnan(_massConcentrationPm1p0) ||
        _massConcentrationPm1p0 == OVERFLOW_SEN6X) {
      return false;
    }
    pm10StdEvent->pm10_std = _massConcentrationPm1p0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    if (!ReadSensorData() || isnan(_massConcentrationPm2p5) ||
        _massConcentrationPm2p5 == OVERFLOW_SEN6X) {
      return false;
    }
    pm25StdEvent->pm25_std = _massConcentrationPm2p5;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    if (!ReadSensorData() || isnan(_massConcentrationPm10p0) ||
        _massConcentrationPm10p0 == OVERFLOW_SEN6X) {
      return false;
    }
    pm100StdEvent->pm100_std = _massConcentrationPm10p0;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the SEN6X sensor's CO2 reading.
      @param    co2Event
                  Adafruit Sensor event for CO2
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    if (!ReadSensorData() || _co2 == 0xFFFF) {
      return false;
    }
    co2Event->CO2 = _co2;
    return true;
  }

protected:
  SensirionI2cSen66 *_sen = nullptr; ///< SEN6X driver object
  float _massConcentrationPm1p0;     ///< PM1.0 mass concentration
  float _massConcentrationPm2p5;     ///< PM2.5 mass concentration
  float _massConcentrationPm4p0;     ///< PM4.0 mass concentration
  float _massConcentrationPm10p0;    ///< PM10.0 mass concentration
  float _ambientHumidity;            ///< Ambient humidity
  float _ambientTemperature;         ///< Ambient temperature
  float _vocIndex;                   ///< VOC index
  float _noxIndex;                   ///< NOx index
  uint16_t _co2;                     ///< CO2 value
};

#endif // DRV_SEN6X_H
