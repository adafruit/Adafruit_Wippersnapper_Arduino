/*!
 * @file drvIna260.cpp
 *
 * Device driver for the INA260 DC Current and Voltage Monitor
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

#include "drvIna260.h"
#include <Adafruit_INA260.h>

/*******************************************************************************/
/*!
    @brief    Destructor for an INA260 sensor.
*/
/*******************************************************************************/
drvIna260::~drvIna260() {
  if (_ina260) {
    delete _ina260;
    _ina260 = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA260 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna260::begin() {
  _ina260 = new Adafruit_INA260();
  if (!_ina260->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("INA260 failed to initialise!");
    return false;
  }
  return true;
}

/*******************************************************************************/
/*!
    @brief    Configures the INA260 sensor with default settings.
    @returns  True if configured successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna260::configureDefaults() {
  _ina260->setAveragingCount(INA260_COUNT_16);
  _ina260->setVoltageConversionTime(INA260_TIME_140_us);
  _ina260->setCurrentConversionTime(INA260_TIME_140_us);
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
bool drvIna260::setAveragedSamples(const ws_config_Value &averaged_samples) {
  if (averaged_samples.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA260_AveragingCount count;
  switch (averaged_samples.value.int_value) {
  case 0:
    count = INA260_COUNT_1;
    break;
  case 1:
    count = INA260_COUNT_4;
    break;
  case 2:
    count = INA260_COUNT_16;
    break;
  case 3:
    count = INA260_COUNT_64;
    break;
  case 4:
    count = INA260_COUNT_128;
    break;
  case 5:
    count = INA260_COUNT_256;
    break;
  case 6:
    count = INA260_COUNT_512;
    break;
  case 7:
    count = INA260_COUNT_1024;
    break;
  default:
    return false;
  }
  _ina260->setAveragingCount(count);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the bus voltage ADC conversion time setting to the driver.
    @param    voltage_conversion_time
              The voltage conversion time index from the broker
              (0=140us, 1=204us, 2=332us, 3=588us, 4=1.1ms, 5=2.116ms,
              6=4.156ms, 7=8.244ms).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna260::setVoltageConversionTime(
    const ws_config_Value &voltage_conversion_time) {
  if (voltage_conversion_time.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA260_ConversionTime time;
  switch (voltage_conversion_time.value.int_value) {
  case 0:
    time = INA260_TIME_140_us;
    break;
  case 1:
    time = INA260_TIME_204_us;
    break;
  case 2:
    time = INA260_TIME_332_us;
    break;
  case 3:
    time = INA260_TIME_588_us;
    break;
  case 4:
    time = INA260_TIME_1_1_ms;
    break;
  case 5:
    time = INA260_TIME_2_116_ms;
    break;
  case 6:
    time = INA260_TIME_4_156_ms;
    break;
  case 7:
    time = INA260_TIME_8_244_ms;
    break;
  default:
    return false;
  }
  _ina260->setVoltageConversionTime(time);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the shunt/current ADC conversion time setting to the
              driver.
    @param    current_conversion_time
              The current conversion time index from the broker
              (0=140us, 1=204us, 2=332us, 3=588us, 4=1.1ms, 5=2.116ms,
              6=4.156ms, 7=8.244ms).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna260::setCurrentConversionTime(
    const ws_config_Value &current_conversion_time) {
  if (current_conversion_time.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA260_ConversionTime time;
  switch (current_conversion_time.value.int_value) {
  case 0:
    time = INA260_TIME_140_us;
    break;
  case 1:
    time = INA260_TIME_204_us;
    break;
  case 2:
    time = INA260_TIME_332_us;
    break;
  case 3:
    time = INA260_TIME_588_us;
    break;
  case 4:
    time = INA260_TIME_1_1_ms;
    break;
  case 5:
    time = INA260_TIME_2_116_ms;
    break;
  case 6:
    time = INA260_TIME_4_156_ms;
    break;
  case 7:
    time = INA260_TIME_8_244_ms;
    break;
  default:
    return false;
  }
  _ina260->setCurrentConversionTime(time);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the measurement mode setting to the driver.
    @param    mode
              The measurement mode index from the broker
              (0=Shutdown, 1=Triggered, 2=Continuous).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvIna260::setMode(const ws_config_Value &mode) {
  if (mode.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  INA260_MeasurementMode meas_mode;
  switch (mode.value.int_value) {
  case 0:
    meas_mode = INA260_MODE_SHUTDOWN;
    break;
  case 1:
    meas_mode = INA260_MODE_TRIGGERED;
    break;
  case 2:
    meas_mode = INA260_MODE_CONTINUOUS;
    break;
  default:
    return false;
  }
  _ina260->setMode(meas_mode);
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
bool drvIna260::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina260->readBusVoltage() / 1000.0f;
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
bool drvIna260::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina260->readCurrent();
  return true;
}
