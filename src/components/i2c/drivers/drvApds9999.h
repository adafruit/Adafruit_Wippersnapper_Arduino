/*!
 * @file drvApds9999.h
 *
 * Device driver for the APDS-9999 proximity and lux light sensor.
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
#ifndef DRV_APDS9999_H
#define DRV_APDS9999_H

#include "drvBase.h"
#include <Adafruit_APDS9999.h>

/*!
    @brief  Class that provides a driver interface for an APDS-9999 sensor.
*/
class drvApds9999 : public drvBase {
public:
  /*!
      @brief    Constructor for an APDS-9999 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvApds9999(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an APDS-9999 sensor.
  */
  ~drvApds9999() { delete _apds9999; }

  /*!
      @brief    Initializes the APDS-9999 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _apds9999 = new Adafruit_APDS9999();
    if (!_apds9999->begin((uint8_t)_address, _i2c))
      return false;

    // Enable sensors and RGB mode for color data
    _apds9999->enableLightSensor(true);
    _apds9999->enableProximitySensor(true);
    _apds9999->setRGBMode(true);

    return true;
  }

  /*!
      @brief    Applies the APDS-9999's default settings.
      @returns  True if applied successfully, False otherwise.
  */
  bool configureDefaults() override {
    bool is_success = true;
    if (!_apds9999->setLightGain(APDS9999_LIGHT_GAIN_3X))
      is_success = false;
    if (!_apds9999->setLightResolution(APDS9999_LIGHT_RES_18BIT))
      is_success = false;
    if (!_apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_100MS))
      is_success = false;
    if (!_apds9999->setProxResolution(APDS9999_PROX_RES_11BIT))
      is_success = false;
    if (!_apds9999->setProxMeasRate(APDS9999_PROX_RATE_100MS))
      is_success = false;
    return is_success;
  }

  /*!
      @brief    Performs a light sensor read using the Adafruit
                Unified Sensor API.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    uint32_t r, g, b, ir;
    if (!_apds9999->getRGBIRData(&r, &g, &b, &ir))
      return false;

    lightEvent->light = _apds9999->calculateLux(g);
    return true;
  }

  /*!
      @brief    Reads the APDS-9999's proximity value into an event (no unit).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  bool getEventProximity(sensors_event_t *proximityEvent) {
    uint16_t prox;
    if (!_apds9999->readProximity(&prox))
      return false;

    proximityEvent->data[0] = (float)prox;
    return true;
  }

  /*!
      @brief    Applies the ALS (light) gain setting to the driver.
      @param    light_gain
                The light gain index from the broker
                (0=1x, 1=3x, 2=6x, 3=9x, 4=18x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setLightGain(const ws_config_Value &light_gain) override {
    if (light_gain.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = light_gain.value.int_value;
    switch (val) {
    case 0:
      _apds9999->setLightGain(APDS9999_LIGHT_GAIN_1X);
      break;
    case 1:
      _apds9999->setLightGain(APDS9999_LIGHT_GAIN_3X);
      break;
    case 2:
      _apds9999->setLightGain(APDS9999_LIGHT_GAIN_6X);
      break;
    case 3:
      _apds9999->setLightGain(APDS9999_LIGHT_GAIN_9X);
      break;
    case 4:
      _apds9999->setLightGain(APDS9999_LIGHT_GAIN_18X);
      break;
    default:
      return false;
    }
    return true;
  }

  /*!
      @brief    Applies the light resolution setting to the driver.
      @param    light_resolution
                The light resolution index from the broker
                (0=20-bit, 1=19-bit, 2=18-bit, 3=17-bit, 4=16-bit, 5=13-bit).
      @returns  True if applied successfully, False otherwise.
  */
  bool setLightResolution(const ws_config_Value &light_resolution) override {
    if (light_resolution.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = light_resolution.value.int_value;
    switch (val) {
    case 0:
      _apds9999->setLightResolution(APDS9999_LIGHT_RES_20BIT);
      break;
    case 1:
      _apds9999->setLightResolution(APDS9999_LIGHT_RES_19BIT);
      break;
    case 2:
      _apds9999->setLightResolution(APDS9999_LIGHT_RES_18BIT);
      break;
    case 3:
      _apds9999->setLightResolution(APDS9999_LIGHT_RES_17BIT);
      break;
    case 4:
      _apds9999->setLightResolution(APDS9999_LIGHT_RES_16BIT);
      break;
    case 5:
      _apds9999->setLightResolution(APDS9999_LIGHT_RES_13BIT);
      break;
    default:
      return false;
    }
    return true;
  }

  /*!
      @brief    Applies the light measurement rate setting to the driver.
      @param    light_meas_rate
                The light measurement rate index from the broker
                (0=25ms, 1=50ms, 2=100ms, 3=200ms, 4=500ms, 5=1000ms,
                6=2000ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setLightMeasRate(const ws_config_Value &light_meas_rate) override {
    if (light_meas_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = light_meas_rate.value.int_value;
    switch (val) {
    case 0:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_25MS);
      break;
    case 1:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_50MS);
      break;
    case 2:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_100MS);
      break;
    case 3:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_200MS);
      break;
    case 4:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_500MS);
      break;
    case 5:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_1000MS);
      break;
    case 6:
      _apds9999->setLightMeasRate(APDS9999_LIGHT_RATE_2000MS);
      break;
    default:
      return false;
    }
    return true;
  }

  /*!
      @brief    Applies the proximity resolution setting to the driver.
      @param    prox_resolution
                The proximity resolution index from the broker
                (0=8-bit, 1=9-bit, 2=10-bit, 3=11-bit).
      @returns  True if applied successfully, False otherwise.
  */
  bool setProxResolution(const ws_config_Value &prox_resolution) override {
    if (prox_resolution.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = prox_resolution.value.int_value;
    switch (val) {
    case 0:
      _apds9999->setProxResolution(APDS9999_PROX_RES_8BIT);
      break;
    case 1:
      _apds9999->setProxResolution(APDS9999_PROX_RES_9BIT);
      break;
    case 2:
      _apds9999->setProxResolution(APDS9999_PROX_RES_10BIT);
      break;
    case 3:
      _apds9999->setProxResolution(APDS9999_PROX_RES_11BIT);
      break;
    default:
      return false;
    }
    return true;
  }

  /*!
      @brief    Applies the proximity measurement rate setting to the driver.
      @param    prox_meas_rate
                The proximity measurement rate index from the broker
                (0=6.25ms, 1=12.5ms, 2=25ms, 3=50ms, 4=100ms, 5=200ms,
                6=400ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setProxMeasRate(const ws_config_Value &prox_meas_rate) override {
    if (prox_meas_rate.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = prox_meas_rate.value.int_value;
    switch (val) {
    case 0:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_6MS);
      break;
    case 1:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_12MS);
      break;
    case 2:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_25MS);
      break;
    case 3:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_50MS);
      break;
    case 4:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_100MS);
      break;
    case 5:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_200MS);
      break;
    case 6:
      _apds9999->setProxMeasRate(APDS9999_PROX_RATE_400MS);
      break;
    default:
      return false;
    }
    return true;
  }

protected:
  Adafruit_APDS9999 *_apds9999 =
      nullptr; ///< Pointer to APDS-9999 sensor object
};

#endif // DRV_APDS9999_H
