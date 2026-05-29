/*!
 * @file drvVeml7700.h
 *
 * Device driver for the VEML7700 digital luminosity (light) sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2022 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_VEML770_H
#define DRV_VEML770_H

#include "drvBase.h"
#include <Adafruit_VEML7700.h>

/*!
    @brief  Class that provides a driver interface for a VEML7700 sensor.
*/
class drvVeml7700 : public drvBase {
public:
  /*!
      @brief    Constructor for a VEML7700 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvVeml7700(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an VEML7700 sensor.
  */
  ~drvVeml7700() { delete _veml; }

  /*!
      @brief    Initializes the VEML7700 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _veml = new Adafruit_VEML7700();
    // Attempt to initialize and configure VEML7700
    return _veml->begin(_i2c);
  }

  /*!
      @brief    Configures the VEML7700 sensor with default settings. The
                VEML7700 powers up in its default configuration, so no
                additional setup is required here.
      @returns  True if configured successfully, False otherwise.
  */
  bool configureDefaults() override { return true; }

  /*!
      @brief    Applies the ALS gain setting to the driver.
      @param    gain
                The gain index from the broker
                (0=1x, 1=2x, 2=1/8x, 3=1/4x).
      @returns  True if applied successfully, False otherwise.
  */
  bool setGain(int32_t gain) override {
    uint8_t als_gain;
    switch (gain) {
    case 0:
      als_gain = VEML7700_GAIN_1;
      break;
    case 1:
      als_gain = VEML7700_GAIN_2;
      break;
    case 2:
      als_gain = VEML7700_GAIN_1_8;
      break;
    case 3:
      als_gain = VEML7700_GAIN_1_4;
      break;
    default:
      als_gain = VEML7700_GAIN_1;
      break;
    }
    _veml->setGain(als_gain);
    return true;
  }

  /*!
      @brief    Applies the ALS integration time setting to the driver.
      @param    integration_time
                The integration time index from the broker
                (0=25ms, 1=50ms, 2=100ms, 3=200ms, 4=400ms, 5=800ms).
      @returns  True if applied successfully, False otherwise.
  */
  bool setIntegrationTime(int32_t integration_time) override {
    uint8_t int_time;
    switch (integration_time) {
    case 0:
      int_time = VEML7700_IT_25MS;
      break;
    case 1:
      int_time = VEML7700_IT_50MS;
      break;
    case 2:
      int_time = VEML7700_IT_100MS;
      break;
    case 3:
      int_time = VEML7700_IT_200MS;
      break;
    case 4:
      int_time = VEML7700_IT_400MS;
      break;
    case 5:
      int_time = VEML7700_IT_800MS;
      break;
    default:
      int_time = VEML7700_IT_100MS;
      break;
    }
    _veml->setIntegrationTime(int_time);
    return true;
  }

  /*!
      @brief    Applies the lux reading method setting to the driver. Selects
                the method used to calculate lux in getEventLight().
      @param    lux_method
                The lux method index from the broker
                (0=Normal, 1=Corrected, 2=Auto, 3=Normal No-Wait,
                4=Corrected No-Wait).
      @returns  True if applied successfully, False otherwise.
  */
  bool setLuxMethod(int32_t lux_method) override {
    switch (lux_method) {
    case 0:
      _luxMethod = VEML_LUX_NORMAL;
      break;
    case 1:
      _luxMethod = VEML_LUX_CORRECTED;
      break;
    case 2:
      _luxMethod = VEML_LUX_AUTO;
      break;
    case 3:
      _luxMethod = VEML_LUX_NORMAL_NOWAIT;
      break;
    case 4:
      _luxMethod = VEML_LUX_CORRECTED_NOWAIT;
      break;
    default:
      _luxMethod = VEML_LUX_NORMAL;
      break;
    }
    return true;
  }

  /*!
      @brief    Performs a light sensor read using the Adafruit
                Unified Sensor API, using the configured lux method.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    // Get sensor event populated in lux via the configured lux method
    lightEvent->light = _veml->readLux(_luxMethod);

    return true;
  }

protected:
  Adafruit_VEML7700 *_veml; ///< Pointer to VEML7700 light sensor object
  luxMethod _luxMethod =
      VEML_LUX_NORMAL; ///< Lux reading method used in getEventLight().
};

#endif // drvVeml7700