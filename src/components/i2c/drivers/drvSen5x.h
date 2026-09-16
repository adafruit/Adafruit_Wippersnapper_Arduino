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

/// Datasheet Table 1: PM number-concentration start-up up to 30s (fan spin-up)
#define SEN5X_PM_STARTUP_MS 30000
/// Datasheet Table 5: VOC/NOx events reliably detected after <60s
#define SEN5X_INDEX_STARTUP_MS 60000
#define SEN5X_STATUS_FAN (1UL << 4)   ///< Device status: fan failure
#define SEN5X_STATUS_LASER (1UL << 5) ///< Device status: laser failure
#define SEN5X_STATUS_RHT (1UL << 6)   ///< Device status: RH/T sensor error
#define SEN5X_STATUS_GAS (1UL << 7)   ///< Device status: gas sensor error
#define SEN5X_STATUS_FAN_CLEANING (1UL << 19) ///< Fan cleaning: values frozen
#define SEN5X_STATUS_SPEED (1UL << 21)        ///< Fan speed out of range

/*!
    @brief  Class that provides a driver interface for the SEN5X sensor.
*/
class drvSen5x : public drvBase {

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
    u_int16_t error_start = _sen->startMeasurement();
    if (error_start != 0) {
      return false;
    }
    // Start-up gates (fan spin-up, gas-index learning) run from here
    _start_ms = millis();
    return true;
  }

  /*!
      @brief    Checks if the sensor has a new measurement ready to read.
      @returns  True if a new measurement is ready, False otherwise.
  */
  bool IsSensorReady() override {
    bool isDataReady = false;
    return (_sen->readDataReady(isDataReady) == 0) && isDataReady;
  }

  /*!
      @brief    Reads all SEN5X metrics in one transaction so every metric in
                a read pass reflects the same sample. The library only writes
                the cached members on success, so the last good sample
                survives a failed read.
      @returns  True if the read succeeded, False otherwise.
  */
  bool ReadSensorData() override {
    if (_sen->readDeviceStatus(_status) != 0)
      return false;
    // Datasheet 5.2: measurement values are not updated during fan cleaning
    if (_status & SEN5X_STATUS_FAN_CLEANING)
      return false;
    return _sen->readMeasuredValues(
               _massConcentrationPm1p0, _massConcentrationPm2p5,
               _massConcentrationPm4p0, _massConcentrationPm10p0,
               _ambientHumidity, _ambientTemperature, _vocIndex,
               _noxIndex) == 0;
  }

  /*!
      @brief    Checks the PM channel is trustworthy: fan spun up (datasheet
                Table 1 start-up, up to 30s) and no fan/laser/speed fault.
      @returns  True if PM values may be published, False otherwise.
  */
  bool PmValid() {
    return millis() - _start_ms >= SEN5X_PM_STARTUP_MS &&
           !(_status &
             (SEN5X_STATUS_FAN | SEN5X_STATUS_LASER | SEN5X_STATUS_SPEED));
  }

  /*!
      @brief    Checks the gas indices are trustworthy: past the switch-on
                learning window (datasheet Table 5, <60s) and no gas fault.
      @returns  True if VOC/NOx indices may be published, False otherwise.
  */
  bool GasValid() {
    return millis() - _start_ms >= SEN5X_INDEX_STARTUP_MS &&
           !(_status & SEN5X_STATUS_GAS);
  }

  /*!
      @brief    Gets the SEN5X's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!AttemptRead() || (_status & SEN5X_STATUS_RHT) ||
        isnan(_ambientTemperature)) {
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
    if (!AttemptRead() || (_status & SEN5X_STATUS_RHT) ||
        isnan(_ambientHumidity)) {
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
    if (!AttemptRead() || !GasValid() || isnan(_noxIndex)) {
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
    if (!AttemptRead() || !GasValid() || isnan(_vocIndex)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm1p0)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm2p5)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm4p0)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm10p0)) {
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
  uint32_t _status = 0;                 ///< Last device status word
  ulong _start_ms = 0;                  ///< millis() measurement started
};

#endif // drvSen5x
