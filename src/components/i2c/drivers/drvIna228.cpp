/*!
 * @file drvIna228.cpp
 *
 * Device driver for the INA228 High Precision DC Current and Voltage Monitor
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

#include "drvIna228.h"
#include <Adafruit_INA228.h>

/*******************************************************************************/
/*!
    @brief    Destructor for an INA228 sensor.
*/
/*******************************************************************************/
drvIna228::~drvIna228() {
  if (_ina228) {
    delete _ina228;
    _ina228 = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA228 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::begin() {
  _ina228 = new Adafruit_INA228();
  if (!_ina228->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("INA228 failed to initialise!");
    return false;
  }
  return true;
}

/*******************************************************************************/
/*!
    @brief    Configures the INA228 sensor with default settings.
    @returns  True if configured successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::configureDefaults() {
  _ina228->setShunt(_shuntResistance, _maxCurrent);
  _ina228->setCurrentConversionTime(INA228_TIME_280_us);
  _ina228->setAveragingCount(INA228_COUNT_16);
  _ina228->setVoltageConversionTime(INA228_TIME_150_us);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the shunt resistance (ohms) calibration value to the
              driver. Recalibrates using the stored max expected current.
    @param    shunt_resistance
              The shunt resistance, in ohms, from the broker (must be > 0).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::setShuntResistance(const ws_config_Value &shunt_resistance) {
  float v;
  if (shunt_resistance.which_value == ws_config_Value_float_value_tag) {
    v = shunt_resistance.value.float_value;
  } else if (shunt_resistance.which_value == ws_config_Value_int_value_tag) {
    v = (float)shunt_resistance.value.int_value;
  } else {
    return false;
  }
  if (v <= 0) {
    return false;
  }
  _shuntResistance = v;
  _ina228->setShunt(_shuntResistance, _maxCurrent);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the maximum expected current (amps) calibration value to
              the driver. Recalibrates using the stored shunt resistance.
    @param    max_current
              The maximum expected current, in amps, from the broker
              (must be > 0).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::setMaxCurrent(const ws_config_Value &max_current) {
  float v;
  if (max_current.which_value == ws_config_Value_float_value_tag) {
    v = max_current.value.float_value;
  } else if (max_current.which_value == ws_config_Value_int_value_tag) {
    v = (float)max_current.value.int_value;
  } else {
    return false;
  }
  if (v <= 0) {
    return false;
  }
  _maxCurrent = v;
  _ina228->setShunt(_shuntResistance, _maxCurrent);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the ADC range setting to the driver (shunt-voltage range).
    @param    adc_range
              The ADC range index from the broker (0=high range, 1=low range).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::setAdcRange(const ws_config_Value &adc_range) {
  if (adc_range.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  int32_t val = adc_range.value.int_value;
  if (val != 0 && val != 1) {
    return false;
  }
  _ina228->setADCRange((uint8_t)val);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the averaging count setting to the driver. Higher counts
              average more samples, trading update rate for lower noise.
    @param    averaged_samples
              The averaging count index from the broker
              (0=1, 1=4, 2=16, 3=64, 4=128, 5=256, 6=512, 7=1024).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::setAveragedSamples(const ws_config_Value &averaged_samples) {
  if (averaged_samples.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA228_AveragingCount count;
  switch (averaged_samples.value.int_value) {
  case 0:
    count = INA228_COUNT_1;
    break;
  case 1:
    count = INA228_COUNT_4;
    break;
  case 2:
    count = INA228_COUNT_16;
    break;
  case 3:
    count = INA228_COUNT_64;
    break;
  case 4:
    count = INA228_COUNT_128;
    break;
  case 5:
    count = INA228_COUNT_256;
    break;
  case 6:
    count = INA228_COUNT_512;
    break;
  case 7:
    count = INA228_COUNT_1024;
    break;
  default:
    return false;
  }
  _ina228->setAveragingCount(count);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the bus voltage ADC conversion time setting to the driver.
    @param    voltage_conversion_time
              The voltage conversion time index from the broker
              (0=50us, 1=84us, 2=150us, 3=280us, 4=540us, 5=1052us, 6=2074us,
              7=4120us).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::setVoltageConversionTime(
    const ws_config_Value &voltage_conversion_time) {
  if (voltage_conversion_time.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA228_ConversionTime time;
  switch (voltage_conversion_time.value.int_value) {
  case 0:
    time = INA228_TIME_50_us;
    break;
  case 1:
    time = INA228_TIME_84_us;
    break;
  case 2:
    time = INA228_TIME_150_us;
    break;
  case 3:
    time = INA228_TIME_280_us;
    break;
  case 4:
    time = INA228_TIME_540_us;
    break;
  case 5:
    time = INA228_TIME_1052_us;
    break;
  case 6:
    time = INA228_TIME_2074_us;
    break;
  case 7:
    time = INA228_TIME_4120_us;
    break;
  default:
    return false;
  }
  _ina228->setVoltageConversionTime(time);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the shunt/current ADC conversion time setting to the
              driver.
    @param    current_conversion_time
              The current conversion time index from the broker
              (0=50us, 1=84us, 2=150us, 3=280us, 4=540us, 5=1052us, 6=2074us,
              7=4120us).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna228::setCurrentConversionTime(
    const ws_config_Value &current_conversion_time) {
  if (current_conversion_time.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA228_ConversionTime time;
  switch (current_conversion_time.value.int_value) {
  case 0:
    time = INA228_TIME_50_us;
    break;
  case 1:
    time = INA228_TIME_84_us;
    break;
  case 2:
    time = INA228_TIME_150_us;
    break;
  case 3:
    time = INA228_TIME_280_us;
    break;
  case 4:
    time = INA228_TIME_540_us;
    break;
  case 5:
    time = INA228_TIME_1052_us;
    break;
  case 6:
    time = INA228_TIME_2074_us;
    break;
  case 7:
    time = INA228_TIME_4120_us;
    break;
  default:
    return false;
  }
  _ina228->setCurrentConversionTime(time);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Reads a voltage sensor and converts the
              reading into the expected SI unit.
    @param    voltageEvent
              voltage sensor reading, in volts.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/*******************************************************************************/
bool drvIna228::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina228->getBusVoltage_V();
  return true;
}

/**
 * @brief   Get the current sensor event.
 *
 * @param   currentEvent  Pointer to the current sensor event.
 *
 * @returns True if the sensor event was obtained successfully, False
 * otherwise.
 */
bool drvIna228::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina228->getCurrent_mA();
  return true;
}
