/*!
 * @file drvStcc4.h
 *
 * Device driver for the STCC4 CO2, Temperature, and Humidity sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_STCC4_H
#define DRV_STCC4_H

#include "drvBase.h"
#include <Adafruit_STCC4.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the STCC4 sensor.
*/
/**************************************************************************/
class drvStcc4 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a STCC4 sensor.
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
  drvStcc4(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
           const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a STCC4 sensor.
  */
  /*******************************************************************************/
  ~drvStcc4() { delete _stcc4; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the STCC4 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _stcc4 = new Adafruit_STCC4();
    if (!_stcc4->begin((uint8_t)_address, _i2c))
      return false;

    // TODO: If device off / idle for long period or reading <3hrs perform
    // conditioning based off of last read time in RTC mem (cleared by hard
    // reset). See datasheet section 3.4.9 - takes 22seconds to complete!

    // Enable continuous measurement mode for periodic reading
    if (!_stcc4->enableContinuousMeasurement(true))
      return false;
    return true;

    // POTENTIAL CUSTOM SETTINGS (not yet exposed via the v2 properties API):
    //  - Ambient pressure / RH-T compensation for accurate CO2 readings
    //    (setAmbientPressure / RHT compensation), from a paired sensor.
    //    -- For adafruit STCC4 breakout it uses onboard SHT4x, but support
    //    others.
    //  - Automatic self-calibration (ASC) enable/disable.
    //    -- Yes we want this option exposed!
    //  - Single-shot vs continuous measurement mode.
    //    -- Continuous is default, but single-shot is available for sleep-mode.
  }

  /*******************************************************************************/
  /*!
      @brief    Reads all sensor data from the STCC4 in one transaction,
                caching the results so temp/humidity/CO2 stay in sync. Serves
                the cached sample if the last read was under one second ago.
      @returns  True if a valid sample is cached, False if no sample has been
                read yet (or the read failed).
  */
  /*******************************************************************************/
  bool ReadSensorData() override {
    if (HasBeenReadInLastSecond())
      return _have_data;

    uint16_t co2, status;
    float temperature, humidity;
    if (_stcc4->readMeasurement(&co2, &temperature, &humidity, &status)) {
      _cachedCO2 = co2;
      _cachedTemperature = temperature;
      _cachedHumidity = humidity;
      _last_read = millis();
      _have_data = true;
    }
    return _have_data;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current temperature.
      @param    tempEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the temperature was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventAmbientTemp(sensors_event_t *tempEvent) {
    if (!ReadSensorData())
      return false;
    tempEvent->temperature = _cachedTemperature;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current relative humidity reading.
      @param    humidEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the humidity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRelativeHumidity(sensors_event_t *humidEvent) {
    if (!ReadSensorData())
      return false;
    humidEvent->relative_humidity = _cachedHumidity;
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the STCC4's current CO2 reading.
      @param    co2Event
                Pointer to an Adafruit_Sensor event.
      @returns  True if the CO2 value was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventCO2(sensors_event_t *co2Event) {
    if (!ReadSensorData())
      return false;
    co2Event->CO2 = (float)_cachedCO2;
    return true;
  }

protected:
  Adafruit_STCC4 *_stcc4 = nullptr; ///< STCC4 driver object
  float _cachedTemperature = NAN;   ///< Cached temperature reading
  float _cachedHumidity = NAN;      ///< Cached humidity reading
  uint16_t _cachedCO2 = 0;          ///< Cached CO2 reading in ppm
};

#endif // DRV_STCC4_H
