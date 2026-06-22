/*!
 * @file drvVcnl4030.h
 *
 * Device driver for the VCNL4030 proximity + ambient light sensor.
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
#ifndef DRV_VCNL4030_H
#define DRV_VCNL4030_H

#include "drvBase.h"
#include <Adafruit_VCNL4030.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4030 sensor.
*/
/**************************************************************************/
class drvVcnl4030 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4030 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  /*******************************************************************************/
  drvVcnl4030(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4030 sensor.
  */
  /*******************************************************************************/
  ~drvVcnl4030() { delete _vcnl4030; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VCNL4030 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    if (_vcnl4030) {
      delete _vcnl4030;
    }
    _vcnl4030 = new Adafruit_VCNL4030();
    if (!_vcnl4030->begin(_address, _i2c))
      return false;

    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the VCNL4030 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    return _vcnl4030->enableALS(true) && _vcnl4030->enableProx(true) &&
           _vcnl4030->setProxLEDCurrent(VCNL4030_PROX_LED_200MA) &&
           _vcnl4030->setProxDuty(VCNL4030_PROX_DUTY_40) &&
           _vcnl4030->setProxIntegrationTime(VCNL4030_PROX_IT_8T) &&
           _vcnl4030->setProxResolution16Bit(true) &&
           _vcnl4030->setALSIntegrationTime(VCNL4030_ALS_IT_100MS);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the ALS integration time setting to the driver. Longer
                integration times give higher lux resolution but slower updates.
      @param    integration_time
                The ALS integration time index from the broker
                (0=50ms, 1=100ms, 2=200ms, 3=400ms, 4=800ms).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setIntegrationTime(const ws_config_Value &integration_time) override {
    if (integration_time.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4030_als_it_t it;
    switch (integration_time.value.int_value) {
    case 0:
      it = VCNL4030_ALS_IT_50MS;
      break;
    case 1:
      it = VCNL4030_ALS_IT_100MS;
      break;
    case 2:
      it = VCNL4030_ALS_IT_200MS;
      break;
    case 3:
      it = VCNL4030_ALS_IT_400MS;
      break;
    case 4:
      it = VCNL4030_ALS_IT_800MS;
      break;
    default:
      return false;
    }
    return _vcnl4030->setALSIntegrationTime(it);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the proximity LED current setting to the driver. Higher
                current gives longer detection range at the cost of more power.
      @param    prox_led_current
                The proximity LED current index from the broker
                (0=50mA, 1=75mA, 2=100mA, 3=120mA, 4=140mA, 5=160mA, 6=180mA,
                7=200mA).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setProxLedCurrent(const ws_config_Value &prox_led_current) override {
    if (prox_led_current.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4030_prox_led_t current;
    switch (prox_led_current.value.int_value) {
    case 0:
      current = VCNL4030_PROX_LED_50MA;
      break;
    case 1:
      current = VCNL4030_PROX_LED_75MA;
      break;
    case 2:
      current = VCNL4030_PROX_LED_100MA;
      break;
    case 3:
      current = VCNL4030_PROX_LED_120MA;
      break;
    case 4:
      current = VCNL4030_PROX_LED_140MA;
      break;
    case 5:
      current = VCNL4030_PROX_LED_160MA;
      break;
    case 6:
      current = VCNL4030_PROX_LED_180MA;
      break;
    case 7:
      current = VCNL4030_PROX_LED_200MA;
      break;
    default:
      return false;
    }
    return _vcnl4030->setProxLEDCurrent(current);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the proximity duty cycle setting to the driver.
      @param    prox_duty
                The proximity duty cycle index from the broker
                (0=1/40, 1=1/80, 2=1/160, 3=1/320).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setProxDuty(const ws_config_Value &prox_duty) override {
    if (prox_duty.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4030_prox_duty_t duty;
    switch (prox_duty.value.int_value) {
    case 0:
      duty = VCNL4030_PROX_DUTY_40;
      break;
    case 1:
      duty = VCNL4030_PROX_DUTY_80;
      break;
    case 2:
      duty = VCNL4030_PROX_DUTY_160;
      break;
    case 3:
      duty = VCNL4030_PROX_DUTY_320;
      break;
    default:
      return false;
    }
    return _vcnl4030->setProxDuty(duty);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the proximity integration time setting to the driver.
      @param    prox_integration_time
                The proximity integration time index from the broker
                (0=1T, 1=1.5T, 2=2T, 3=2.5T, 4=3T, 5=3.5T, 6=4T, 7=8T).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setProxIntegrationTime(
      const ws_config_Value &prox_integration_time) override {
    if (prox_integration_time.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4030_prox_it_t it;
    switch (prox_integration_time.value.int_value) {
    case 0:
      it = VCNL4030_PROX_IT_1T;
      break;
    case 1:
      it = VCNL4030_PROX_IT_1_5T;
      break;
    case 2:
      it = VCNL4030_PROX_IT_2T;
      break;
    case 3:
      it = VCNL4030_PROX_IT_2_5T;
      break;
    case 4:
      it = VCNL4030_PROX_IT_3T;
      break;
    case 5:
      it = VCNL4030_PROX_IT_3_5T;
      break;
    case 6:
      it = VCNL4030_PROX_IT_4T;
      break;
    case 7:
      it = VCNL4030_PROX_IT_8T;
      break;
    default:
      return false;
    }
    return _vcnl4030->setProxIntegrationTime(it);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the proximity resolution setting to the driver.
      @param    prox_resolution
                The proximity resolution index from the broker
                (0=12-bit, 1=16-bit).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setProxResolution(const ws_config_Value &prox_resolution) override {
    if (prox_resolution.which_value != ws_config_Value_int_value_tag)
      return false;
    bool enable;
    switch (prox_resolution.value.int_value) {
    case 0:
      enable = false;
      break;
    case 1:
      enable = true;
      break;
    default:
      return false;
    }
    return _vcnl4030->setProxResolution16Bit(enable);
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VCNL4030's current light reading in lux.
      @param    lightEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the light reading was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    lightEvent->light = _vcnl4030->readLux();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Gets the VCNL4030's current proximity reading.
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->data[0] = (float)_vcnl4030->readProximity();
    return true;
  }

protected:
  Adafruit_VCNL4030 *_vcnl4030 = nullptr; ///< Pointer to VCNL4030 sensor object
};

#endif // DRV_VCNL4030_H
