/*!
 * @file drvAs5600.h
 *
 * Device driver for the AS5600 Magnetic Angle sensor.
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
#ifndef DRV_AS5600_H
#define DRV_AS5600_H

#include "drvBase.h"
#include <Adafruit_AS5600.h>

/*!
    @brief  Class that provides a driver interface for an AS5600 sensor.
*/
class drvAs5600 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for the AS5600 sensor.
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
  drvAs5600(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    _as5600 = nullptr;
    _angle = 0;
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for an AS5600 sensor.
  */
  /*******************************************************************************/
  ~drvAs5600() {
    if (_as5600) {
      delete _as5600;
      _as5600 = nullptr;
    }
  }

  /*******************************************************************************/
  /*!
      @brief    Initializes the AS5600 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _as5600 = new Adafruit_AS5600();
    if (!_as5600->begin((uint8_t)_address, _i2c)) {
      WS_DEBUG_PRINTLN("Failed to find AS5600 chip");
      return false;
    }
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Configures the AS5600 sensor with default settings.
      @returns  True if the sensor was configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override {
    bool ok = true;
    ok = ok && _as5600->enableWatchdog(false);
    // Normal (high) power mode
    ok = ok && _as5600->setPowerMode(AS5600_POWER_MODE_NOM);
    // No Hysteresis
    ok = ok && _as5600->setHysteresis(AS5600_HYSTERESIS_OFF);
    // analog output (0-VCC for 0-360 degrees)
    ok = ok && _as5600->setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);
    // setup filters
    ok = ok && _as5600->setSlowFilter(AS5600_SLOW_FILTER_16X);
    ok =
        ok && _as5600->setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);
    // Reset position settings to defaults
    ok = ok && _as5600->setZPosition(0);
    ok = ok && _as5600->setMPosition(4095);
    ok = ok && _as5600->setMaxAngle(4095);
    return ok;
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the output stage setting to the driver. The output stage
                selects how the angle is presented on the OUT pin.
      @param    output_stage
                The output stage index from the broker
                (0=Analog Full 0-100%, 1=Analog Reduced 10-90%, 2=Digital PWM).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setOutputStage(const ws_config_Value &output_stage) override {
    if (output_stage.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    as5600_output_stage_t stage;
    switch (output_stage.value.int_value) {
    case 0:
      stage = AS5600_OUTPUT_STAGE_ANALOG_FULL;
      break;
    case 1:
      stage = AS5600_OUTPUT_STAGE_ANALOG_REDUCED;
      break;
    case 2:
      stage = AS5600_OUTPUT_STAGE_DIGITAL_PWM;
      break;
    default:
      return false;
    }
    return _as5600->setOutputStage(stage);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the power mode setting to the driver. Lower power modes
                trade response time for reduced current draw.
      @param    power_mode
                The power mode index from the broker
                (0=Normal, 1=Low Power 1, 2=Low Power 2, 3=Low Power 3).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setPowerMode(const ws_config_Value &power_mode) override {
    if (power_mode.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    as5600_power_mode_t mode;
    switch (power_mode.value.int_value) {
    case 0:
      mode = AS5600_POWER_MODE_NOM;
      break;
    case 1:
      mode = AS5600_POWER_MODE_LPM1;
      break;
    case 2:
      mode = AS5600_POWER_MODE_LPM2;
      break;
    case 3:
      mode = AS5600_POWER_MODE_LPM3;
      break;
    default:
      return false;
    }
    return _as5600->setPowerMode(mode);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the slow (low-pass) filter setting to the driver. Higher
                averaging reduces noise at the cost of responsiveness.
      @param    slow_filter
                The slow filter index from the broker
                (0=16x, 1=8x, 2=4x, 3=2x).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setSlowFilter(const ws_config_Value &slow_filter) override {
    if (slow_filter.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    as5600_slow_filter_t filter;
    switch (slow_filter.value.int_value) {
    case 0:
      filter = AS5600_SLOW_FILTER_16X;
      break;
    case 1:
      filter = AS5600_SLOW_FILTER_8X;
      break;
    case 2:
      filter = AS5600_SLOW_FILTER_4X;
      break;
    case 3:
      filter = AS5600_SLOW_FILTER_2X;
      break;
    default:
      return false;
    }
    return _as5600->setSlowFilter(filter);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the fast filter threshold setting to the driver. The
     fast filter kicks in when the angle change exceeds this threshold.
      @param    fast_filter_threshold
                The fast filter threshold index from the broker
                (0=Slow Filter Only, 1=6 LSB, 2=7 LSB, 3=9 LSB, 4=18 LSB,
                5=21 LSB, 6=24 LSB, 7=10 LSB).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setFastFilterThreshold(
      const ws_config_Value &fast_filter_threshold) override {
    if (fast_filter_threshold.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    as5600_fast_filter_thresh_t thresh;
    switch (fast_filter_threshold.value.int_value) {
    case 0:
      thresh = AS5600_FAST_FILTER_THRESH_SLOW_ONLY;
      break;
    case 1:
      thresh = AS5600_FAST_FILTER_THRESH_6LSB;
      break;
    case 2:
      thresh = AS5600_FAST_FILTER_THRESH_7LSB;
      break;
    case 3:
      thresh = AS5600_FAST_FILTER_THRESH_9LSB;
      break;
    case 4:
      thresh = AS5600_FAST_FILTER_THRESH_18LSB;
      break;
    case 5:
      thresh = AS5600_FAST_FILTER_THRESH_21LSB;
      break;
    case 6:
      thresh = AS5600_FAST_FILTER_THRESH_24LSB;
      break;
    case 7:
      thresh = AS5600_FAST_FILTER_THRESH_10LSB;
      break;
    default:
      return false;
    }
    return _as5600->setFastFilterThresh(thresh);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the hysteresis setting to the driver. Hysteresis reduces
                output jitter when the magnet is stationary.
      @param    hysteresis
                The hysteresis index from the broker
                (0=Off, 1=1 LSB, 2=2 LSB, 3=3 LSB).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setHysteresis(const ws_config_Value &hysteresis) override {
    if (hysteresis.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    as5600_hysteresis_t hyst;
    switch (hysteresis.value.int_value) {
    case 0:
      hyst = AS5600_HYSTERESIS_OFF;
      break;
    case 1:
      hyst = AS5600_HYSTERESIS_1LSB;
      break;
    case 2:
      hyst = AS5600_HYSTERESIS_2LSB;
      break;
    case 3:
      hyst = AS5600_HYSTERESIS_3LSB;
      break;
    default:
      return false;
    }
    return _as5600->setHysteresis(hyst);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the zero (start) position to the driver. Sets the raw
                angle (0..4095) that maps to the start of the output range.
      @param    z_position
                The zero position from the broker (valid range 0..4095).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setZPosition(const ws_config_Value &z_position) override {
    if (z_position.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = z_position.value.int_value;
    if (val < 0 || val > 4095) {
      return false;
    }
    return _as5600->setZPosition((uint16_t)val);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the maximum (stop) position to the driver. Sets the raw
                angle (0..4095) that maps to the end of the output range.
      @param    m_position
                The maximum position from the broker (valid range 0..4095).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMPosition(const ws_config_Value &m_position) override {
    if (m_position.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = m_position.value.int_value;
    if (val < 0 || val > 4095) {
      return false;
    }
    return _as5600->setMPosition((uint16_t)val);
  }

  /*******************************************************************************/
  /*!
      @brief    Applies the maximum angle to the driver. Sets the angular span
                (0..4095) mapped across the full output range.
      @param    max_angle
                The maximum angle from the broker (valid range 0..4095).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMaxAngle(const ws_config_Value &max_angle) override {
    if (max_angle.which_value != ws_config_Value_int_value_tag) {
      return false;
    }
    int32_t val = max_angle.value.int_value;
    if (val < 0 || val > 4095) {
      return false;
    }
    return _as5600->setMaxAngle((uint16_t)val);
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the Angle sensor.
      @returns  True if the sensor was read successfully, False otherwise.
  */
  /*******************************************************************************/
  bool readSensor() {
    if (!_as5600->isMagnetDetected()) {
      WS_DEBUG_PRINTLN("Magnet not detected!");
      return false;
    }
    if (_as5600->isAGCminGainOverflow()) {
      WS_DEBUG_PRINTLN("MH: magnet too strong");
    } else if (_as5600->isAGCmaxGainOverflow()) {
      WS_DEBUG_PRINTLN("ML: magnet too weak");
    } else {
      uint16_t angle = _as5600->getAngle();
      _angle = ((float)angle / 4095.0) * 360.0;
      return true;
    }
    // Show raw changing data if low/high value errors
    uint16_t rawAngle = _as5600->getRawAngle();
    WS_DEBUG_PRINT("AS5600 Raw: ");
    WS_DEBUG_PRINTLNVAR(rawAngle);
    return false;
  }

  /*******************************************************************************/
  /*!
      @brief    Reads the Angle sensor with short wait for data.
      @param    rawEvent
                Angle sensor reading
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!readSensor()) {
      return false;
    }
    rawEvent->data[0] = _angle;
    return true;
  }

protected:
  float _angle;             ///< Current angle reading from the AS5600 sensor
  Adafruit_AS5600 *_as5600; ///< Pointer to AS5600 sensor object
};

#endif // DRV_AS5600_H
