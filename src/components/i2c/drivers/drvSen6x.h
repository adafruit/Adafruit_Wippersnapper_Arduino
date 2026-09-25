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

/// Datasheet 4.3.3: fan speed is unchecked for the first 10s (settling); PM
/// start-up as SEN5x Table 1, up to 30s
#define SEN6X_PM_STARTUP_MS 30000
/// Datasheet Table 4: VOC/NOx events reliably detected after <60s
#define SEN6X_INDEX_STARTUP_MS 60000

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the SEN6X sensor.
*/
/**************************************************************************/
class drvSen6x : public drvBase {

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
    uint16_t error_start = _sen->startContinuousMeasurement();
    if (error_start != 0) {
      return false;
    }
    // Start-up gates (fan settling, gas-index learning) run from here
    _start_ms = millis();
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
  bool IsSensorReady() override {
    bool isDataReady = false;
    uint8_t padding = 0x0;
    return (_sen->getDataReady(padding, isDataReady) == 0) && isDataReady;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads all SEN6X metrics in one transaction so every metric in
                a read pass reflects the same sample. The library only writes
                the cached members on success, so the last good sample
                survives a failed read.
      @returns  True if the read succeeded, False otherwise.
  */
  /*******************************************************************************/
  bool ReadSensorData() override {
    if (_sen->readDeviceStatus(_status) != 0)
      return false;
    uint16_t pm1, pm25, pm4, pm10, co2;
    int16_t rh, t, voc, nox;
    if (_sen->readMeasuredValuesAsIntegers(pm1, pm25, pm4, pm10, rh, t, voc,
                                           nox, co2) != 0)
      return false;
    // Datasheet 4.8.8: 0xFFFF (uint16) / 0x7FFF (int16) mean "unknown" - e.g.
    // CO2 for the first 5-6s, VOC/NOx for the first 10-11s. The SEN66 library
    // returns them unscaled (3276.7, 6553.5 ...), so map them to NAN here.
    _massConcentrationPm1p0 = pm1 == 0xFFFF ? NAN : pm1 / 10.0f;
    _massConcentrationPm2p5 = pm25 == 0xFFFF ? NAN : pm25 / 10.0f;
    _massConcentrationPm4p0 = pm4 == 0xFFFF ? NAN : pm4 / 10.0f;
    _massConcentrationPm10p0 = pm10 == 0xFFFF ? NAN : pm10 / 10.0f;
    _ambientHumidity = rh == 0x7FFF ? NAN : rh / 100.0f;
    _ambientTemperature = t == 0x7FFF ? NAN : t / 200.0f;
    _vocIndex = voc == 0x7FFF ? NAN : voc / 10.0f;
    _noxIndex = nox == 0x7FFF ? NAN : nox / 10.0f;
    _co2 = co2;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks the PM channel is trustworthy: fan settled (datasheet
                4.3.3 / SEN5x Table 1 start-up) and no fan/PM/speed fault.
      @returns  True if PM values may be published, False otherwise.
  */
  /*******************************************************************************/
  bool PmValid() {
    return millis() - _start_ms >= SEN6X_PM_STARTUP_MS && !_status.fanError &&
           !_status.pmError && !_status.fanSpeedWarning;
  }

  /*******************************************************************************/
  /*!
      @brief    Checks the gas indices are trustworthy: past the switch-on
                learning window (datasheet Table 4, <60s) and no gas fault.
      @returns  True if VOC/NOx indices may be published, False otherwise.
  */
  /*******************************************************************************/
  bool GasValid() {
    return millis() - _start_ms >= SEN6X_INDEX_STARTUP_MS && !_status.gasError;
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
    if (!AttemptRead() || _status.rhtError || isnan(_ambientTemperature)) {
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
    if (!AttemptRead() || _status.rhtError || isnan(_ambientHumidity)) {
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
    if (!AttemptRead() || !GasValid() || isnan(_noxIndex)) {
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
    if (!AttemptRead() || !GasValid() || isnan(_vocIndex)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm1p0)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm2p5)) {
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
    if (!AttemptRead() || !PmValid() || isnan(_massConcentrationPm10p0)) {
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
    // 0xFFFF = unknown (first 5-6s); co22Error = CO2 sensor fault
    if (!AttemptRead() || _co2 == 0xFFFF || _status.co22Error) {
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
  SEN66DeviceStatus _status = {};    ///< Last device status word
  ulong _start_ms = 0;               ///< millis() measurement started
};

#endif // DRV_SEN6X_H
