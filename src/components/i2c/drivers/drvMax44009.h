/*!
 * @file drvMax44009.h
 *
 * Device driver for the MAX44009 ambient light sensor.
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
#ifndef DRV_MAX44009_H
#define DRV_MAX44009_H

#include "drvBase.h"
#include <Adafruit_MAX44009.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for the MAX44009
            ambient light sensor.
*/
/**************************************************************************/
class drvMax44009 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a MAX44009 sensor.
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
  drvMax44009(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a MAX44009 sensor.
  */
  /*******************************************************************************/
  ~drvMax44009() { delete _max44009; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the MAX44009 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_max44009) {
      delete _max44009;
    }
    _max44009 = new Adafruit_MAX44009();
    return _max44009->begin((uint8_t)_address, _i2c);
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the MAX44009 sensor with default settings.
                Explicitly sets the default (auto-ranging, 800ms cycle) mode so
                a future library change cannot silently alter WipperSnapper
                behavior.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    _max44009->setMode(MAX44009_MODE_DEFAULT);
    if (!(_max44009->getMode() == MAX44009_MODE_DEFAULT)) {
      WS_DEBUG_PRINTLN("Failed to set MAX44009 mode!");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the operating mode setting to the driver.
      @param    mode
                The mode index from the broker
                (0=Default auto 800ms, 1=Continuous auto, 2=Manual 800ms,
                3=Manual continuous).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMode(const ws_config_Value &mode) override {
    if (mode.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    max44009_mode_t meas_mode;
    switch (mode.value.int_value) {
    case 0:
      meas_mode = MAX44009_MODE_DEFAULT;
      break;
    case 1:
      meas_mode = MAX44009_MODE_CONTINUOUS;
      break;
    case 2:
      meas_mode = MAX44009_MODE_MANUAL;
      break;
    case 3:
      meas_mode = MAX44009_MODE_MANUAL_CONTINUOUS;
      break;
    default:
      return false;
    }
    _max44009->setMode(meas_mode);
    return _max44009->getMode() == meas_mode;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the ALS integration time setting to the driver. Shorter
                times (50ms and below) require a manual mode.
      @param    integration_time
                The integration time index from the broker
                (0=800ms, 1=400ms, 2=200ms, 3=100ms, 4=50ms, 5=25ms,
                6=12.5ms, 7=6.25ms).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setIntegrationTime(const ws_config_Value &integration_time) override {
    if (integration_time.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    max44009_integration_time_t integ;
    switch (integration_time.value.int_value) {
    case 0:
      integ = MAX44009_INTEGRATION_800MS;
      break;
    case 1:
      integ = MAX44009_INTEGRATION_400MS;
      break;
    case 2:
      integ = MAX44009_INTEGRATION_200MS;
      break;
    case 3:
      integ = MAX44009_INTEGRATION_100MS;
      break;
    case 4:
      integ = MAX44009_INTEGRATION_50MS;
      break;
    case 5:
      integ = MAX44009_INTEGRATION_25MS;
      break;
    case 6:
      integ = MAX44009_INTEGRATION_12_5MS;
      break;
    case 7:
      integ = MAX44009_INTEGRATION_6_25MS;
      break;
    default:
      return false;
    }
    _max44009->setIntegrationTime(integ);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the MAX44009's current ambient light reading in lux.
      @param    luxEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLux(sensors_event_t *luxEvent) {
    float lux = _max44009->readLux();
    if (isnan(lux))
      return false;
    luxEvent->light = lux;
    return true;
  }

protected:
  Adafruit_MAX44009 *_max44009 = nullptr; ///< Pointer to MAX44009 sensor object
};

#endif // DRV_MAX44009_H
