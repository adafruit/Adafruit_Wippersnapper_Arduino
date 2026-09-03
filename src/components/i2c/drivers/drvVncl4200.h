/*!
 * @file drvVncl4200.h
 *
 * Device driver for the VCNL4200 light + proximity sensor.
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
#ifndef DRV_VCNL4200_H
#define DRV_VCNL4200_H

#include "drvBase.h"
#include <Adafruit_VCNL4200.h>

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a VCNL4200 sensor.
*/
/**************************************************************************/
class drvVncl4200 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a VCNL4200 sensor.
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
  drvVncl4200(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an VCNL4200 sensor.
  */
  /*******************************************************************************/
  ~drvVncl4200() { delete _vcnl4200; }

  /*******************************************************************************/
  /*!
      @brief    Initializes the VCNL4200 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _vcnl4200 = new Adafruit_VCNL4200();
    // Attempt to initialize transport
    if (!_vcnl4200->begin(_address, _i2c)) {
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the VCNL4200 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    return _vcnl4200->setALSshutdown(false) &&
           _vcnl4200->setProxShutdown(false) &&
           _vcnl4200->setProxHD(true) && // 16bit instead of 12bit
           _vcnl4200->setALSIntegrationTime(VCNL4200_ALS_IT_400MS) &&
           _vcnl4200->setProxDuty(VCNL4200_PS_DUTY_1_160) &&
           _vcnl4200->setProxLEDCurrent(VCNL4200_LED_I_200MA) &&
           _vcnl4200->setProxIntegrationTime(VCNL4200_PS_IT_9T);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the ALS integration time setting to the driver. Longer
                integration times give higher lux resolution but slower updates.
      @param    integration_time
                The ALS integration time index from the broker
                (0=50ms, 1=100ms, 2=200ms, 3=400ms).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setIntegrationTime(const ws_config_Value &integration_time) override {
    if (integration_time.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4200_als_it_t it;
    switch (integration_time.value.int_value) {
    case 0:
      it = VCNL4200_ALS_IT_50MS;
      break;
    case 1:
      it = VCNL4200_ALS_IT_100MS;
      break;
    case 2:
      it = VCNL4200_ALS_IT_200MS;
      break;
    case 3:
      it = VCNL4200_ALS_IT_400MS;
      break;
    default:
      return false;
    }
    return _vcnl4200->setALSIntegrationTime(it);
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
    vcnl4200_led_i_t current;
    switch (prox_led_current.value.int_value) {
    case 0:
      current = VCNL4200_LED_I_50MA;
      break;
    case 1:
      current = VCNL4200_LED_I_75MA;
      break;
    case 2:
      current = VCNL4200_LED_I_100MA;
      break;
    case 3:
      current = VCNL4200_LED_I_120MA;
      break;
    case 4:
      current = VCNL4200_LED_I_140MA;
      break;
    case 5:
      current = VCNL4200_LED_I_160MA;
      break;
    case 6:
      current = VCNL4200_LED_I_180MA;
      break;
    case 7:
      current = VCNL4200_LED_I_200MA;
      break;
    default:
      return false;
    }
    return _vcnl4200->setProxLEDCurrent(current);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the proximity duty cycle setting to the driver.
      @param    prox_duty
                The proximity duty cycle index from the broker
                (0=1/160, 1=1/320, 2=1/640, 3=1/1280).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setProxDuty(const ws_config_Value &prox_duty) override {
    if (prox_duty.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4200_ps_duty_t duty;
    switch (prox_duty.value.int_value) {
    case 0:
      duty = VCNL4200_PS_DUTY_1_160;
      break;
    case 1:
      duty = VCNL4200_PS_DUTY_1_320;
      break;
    case 2:
      duty = VCNL4200_PS_DUTY_1_640;
      break;
    case 3:
      duty = VCNL4200_PS_DUTY_1_1280;
      break;
    default:
      return false;
    }
    return _vcnl4200->setProxDuty(duty);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the proximity integration time setting to the driver.
      @param    prox_integration_time
                The proximity integration time index from the broker
                (0=1T, 1=2T, 2=3T, 3=4T, 4=8T, 5=9T).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setProxIntegrationTime(
      const ws_config_Value &prox_integration_time) override {
    if (prox_integration_time.which_value != ws_config_Value_int_value_tag)
      return false;
    vcnl4200_ps_it_t it;
    switch (prox_integration_time.value.int_value) {
    case 0:
      it = VCNL4200_PS_IT_1T;
      break;
    case 1:
      it = VCNL4200_PS_IT_2T;
      break;
    case 2:
      it = VCNL4200_PS_IT_3T;
      break;
    case 3:
      it = VCNL4200_PS_IT_4T;
      break;
    case 4:
      it = VCNL4200_PS_IT_8T;
      break;
    case 5:
      it = VCNL4200_PS_IT_9T;
      break;
    default:
      return false;
    }
    return _vcnl4200->setProxIntegrationTime(it);
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
    bool high;
    switch (prox_resolution.value.int_value) {
    case 0:
      high = false;
      break;
    case 1:
      high = true;
      break;
    default:
      return false;
    }
    return _vcnl4200->setProxHD(high);
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a light sensor read using the Adafruit
                Unified Sensor API.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    // Get sensor event populated in lux via AUTO integration and gain
    lightEvent->light = _vcnl4200->readALSdata();
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the VCNL4200's proximity value into an event (no unit).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventProximity(sensors_event_t *proximityEvent) {
    proximityEvent->data[0] = (float)_vcnl4200->readProxData();
    return true;
  }

protected:
  Adafruit_VCNL4200 *_vcnl4200; ///< Pointer to VCNL4200 light sensor object
};

#endif // DRV_VCNL4200_H
