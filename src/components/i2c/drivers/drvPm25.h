/*!
 * @file drvPm25.h
 *
 * I2C driver for the Adafruit PM2.5 Air Quality Sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022-2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_PM25_H
#define DRV_PM25_H

#include "drvBase.h"
#include <Adafruit_PM25AQI.h>

/// Plantower manual: "stable data should be got at least 30 seconds after the
/// sensor wakeup ... because of the fan's performance"
#define PM25_FAN_STARTUP_MS 30000
#include <Wire.h>

/*!
    @brief  Class that provides a driver interface for the PM25 sensor.
*/
class drvPm25 : public drvBase {

public:
  /*!
      @brief    Constructor for a PM25 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvPm25(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
          const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Initializes the PM25 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _pm25 = new Adafruit_PM25AQI();
    if (!_pm25->begin_I2C(_i2c))
      return false;
    // The fan spin-up gate in IsSensorReady() runs from here
    _boot_ms = millis();
    return true;
  }

  /*!
      @brief    Waits out the fan spin-up: the manual says data is stable at
                least 30s after wake-up.
      @returns  True once frames can be trusted, False otherwise.
  */
  bool IsSensorReady() override {
    return millis() - _boot_ms >= PM25_FAN_STARTUP_MS;
  }

  /*!
      @brief    Reads one PM2.5 AQI data frame so the PM1.0/2.5/10 metrics in
                a read pass come from the same sample.
      @returns  True if a frame was read successfully, False otherwise.
  */
  bool ReadSensorData() override {
    if (!_pm25->read(&_data)) {
      WS_DEBUG_PRINTLN("Failed to read PM25 data frame");
      return false;
    }
    // Frame data 13: high byte firmware version, low byte error code
    if ((_data.unused & 0x00FF) != 0) {
      WS_DEBUG_PRINT("PM25 frame reports error code ");
      WS_DEBUG_PRINTLNVAR(_data.unused & 0x00FF);
      return false;
    }
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM1.0 STD reading.
      @param    pm10StdEvent
                  Adafruit Sensor event for PM1.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM10_STD(sensors_event_t *pm10StdEvent) {
    if (!AttemptRead())
      return false;
    pm10StdEvent->pm10_std = (float)_data.pm10_standard;
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM2.5 STD reading.
      @param    pm25StdEvent
                  Adafruit Sensor event for PM2.5
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM25_STD(sensors_event_t *pm25StdEvent) {
    if (!AttemptRead())
      return false;
    pm25StdEvent->pm25_std = (float)_data.pm25_standard;
    return true;
  }

  /*!
      @brief    Gets the PM25 sensor's PM10.0 STD reading.
      @param    pm100StdEvent
                  Adafruit Sensor event for PM10.0
      @returns  True if the sensor value was obtained successfully, False
                otherwise.
  */
  bool getEventPM100_STD(sensors_event_t *pm100StdEvent) {
    if (!AttemptRead())
      return false;
    pm100StdEvent->pm100_std = (float)_data.pm100_standard;
    return true;
  }

protected:
  Adafruit_PM25AQI *_pm25;   ///< PM25 driver object
  PM25_AQI_Data _data = {0}; ///< Cached data frame from the last read
  ulong _boot_ms = 0;        ///< millis() the sensor was started
};

#endif // drvPm25