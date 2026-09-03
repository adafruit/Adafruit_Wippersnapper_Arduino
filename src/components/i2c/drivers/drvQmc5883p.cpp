/*!
 * @file drvQmc5883p.cpp
 *
 * Driver wrapper for the Adafruit QMC5883P 3-axis magnetometer.
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

#include "drvQmc5883p.h"

#include <Adafruit_QMC5883P.h>

/*******************************************************************************/
/*!
    @brief    Destructor for a QMC5883P sensor.
*/
/*******************************************************************************/
drvQmc5883p::~drvQmc5883p() {
  if (_qmc) {
    delete _qmc;
    _qmc = nullptr;
  }
}

/*******************************************************************************/
/*!
    @brief    Initializes the QMC5883P sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::begin() {
  _qmc = new Adafruit_QMC5883P();
  if (!_qmc->begin(_address, _i2c)) {
    WS_DEBUG_PRINTLN("QMC5883P failed to initialise!");
    return false;
  }
  return true;
}

/*******************************************************************************/
/*!
    @brief    Configures the QMC5883P sensor with default settings.
    @returns  True if configured successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::configureDefaults() {
  _qmc->setMode(QMC5883P_MODE_CONTINUOUS);
  _qmc->setODR(QMC5883P_ODR_50HZ);
  _qmc->setOSR(QMC5883P_OSR_4);
  _qmc->setDSR(QMC5883P_DSR_2);
  _qmc->setRange(QMC5883P_RANGE_30G);
  _qmc->setSetResetMode(QMC5883P_SETRESET_ON);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the operating mode setting to the driver.
    @param    mode
              The mode index from the broker
              (0=Suspend, 1=Normal, 2=Single, 3=Continuous).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::setMode(const ws_config_Value &mode) {
  if (mode.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  qmc5883p_mode_t meas_mode;
  switch (mode.value.int_value) {
  case 0:
    meas_mode = QMC5883P_MODE_SUSPEND;
    break;
  case 1:
    meas_mode = QMC5883P_MODE_NORMAL;
    break;
  case 2:
    meas_mode = QMC5883P_MODE_SINGLE;
    break;
  case 3:
    meas_mode = QMC5883P_MODE_CONTINUOUS;
    break;
  default:
    return false;
  }
  _qmc->setMode(meas_mode);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the output data rate (ODR) setting to the driver.
    @param    output_data_rate
              The output data rate index from the broker
              (0=10Hz, 1=50Hz, 2=100Hz, 3=200Hz).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::setOutputDataRate(const ws_config_Value &output_data_rate) {
  if (output_data_rate.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  qmc5883p_odr_t odr;
  switch (output_data_rate.value.int_value) {
  case 0:
    odr = QMC5883P_ODR_10HZ;
    break;
  case 1:
    odr = QMC5883P_ODR_50HZ;
    break;
  case 2:
    odr = QMC5883P_ODR_100HZ;
    break;
  case 3:
    odr = QMC5883P_ODR_200HZ;
    break;
  default:
    return false;
  }
  _qmc->setODR(odr);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the over-sample ratio (OSR) setting to the driver.
    @param    oversample_ratio
              The over-sample ratio index from the broker
              (0=8, 1=4, 2=2, 3=1).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::setOverSampleRatio(const ws_config_Value &oversample_ratio) {
  if (oversample_ratio.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  qmc5883p_osr_t osr;
  switch (oversample_ratio.value.int_value) {
  case 0:
    osr = QMC5883P_OSR_8;
    break;
  case 1:
    osr = QMC5883P_OSR_4;
    break;
  case 2:
    osr = QMC5883P_OSR_2;
    break;
  case 3:
    osr = QMC5883P_OSR_1;
    break;
  default:
    return false;
  }
  _qmc->setOSR(osr);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the down-sample ratio (DSR) setting to the driver.
    @param    downsample_ratio
              The down-sample ratio index from the broker
              (0=1, 1=2, 2=4, 3=8).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::setDownSampleRatio(const ws_config_Value &downsample_ratio) {
  if (downsample_ratio.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  qmc5883p_dsr_t dsr;
  switch (downsample_ratio.value.int_value) {
  case 0:
    dsr = QMC5883P_DSR_1;
    break;
  case 1:
    dsr = QMC5883P_DSR_2;
    break;
  case 2:
    dsr = QMC5883P_DSR_4;
    break;
  case 3:
    dsr = QMC5883P_DSR_8;
    break;
  default:
    return false;
  }
  _qmc->setDSR(dsr);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Applies the full-scale field range setting to the driver.
    @param    range
              The range index from the broker
              (0=±30 Gauss, 1=±12 Gauss, 2=±8 Gauss, 3=±2 Gauss).
    @returns  True if applied successfully, False otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::setRange(const ws_config_Value &range) {
  if (range.which_value != ws_config_Value_int_value_tag) {
    return false;
  }
  qmc5883p_range_t fs_range;
  switch (range.value.int_value) {
  case 0:
    fs_range = QMC5883P_RANGE_30G;
    break;
  case 1:
    fs_range = QMC5883P_RANGE_12G;
    break;
  case 2:
    fs_range = QMC5883P_RANGE_8G;
    break;
  case 3:
    fs_range = QMC5883P_RANGE_2G;
    break;
  default:
    return false;
  }
  _qmc->setRange(fs_range);
  return true;
}

/*******************************************************************************/
/*!
    @brief    Gets the QMC5883P's magnetic-field magnitude (gauss) as a raw
              event.
    @param    rawEvent
              Pointer to the magnetometer sensor event.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/*******************************************************************************/
bool drvQmc5883p::getEventRaw(sensors_event_t *rawEvent) {
  // Check if data is ready before reading
  if (!_qmc->isDataReady()) {
    return false;
  }

  float gx, gy, gz;

  // Get Gauss field data
  if (!_qmc->getGaussField(&gx, &gy, &gz)) {
    WS_DEBUG_PRINTLN("Failed to read Gauss field data");
    return false;
  }

  // Check for overflow
  if (_qmc->isOverflow()) {
    WS_DEBUG_PRINTLN("QMC5883P data overflow - skipping reading");
    return false;
  }

  WS_DEBUG_PRINT("QMC5883P Gauss X: ");
  WS_DEBUG_PRINTVAR(gx);
  WS_DEBUG_PRINT(" Y: ");
  WS_DEBUG_PRINTVAR(gy);
  WS_DEBUG_PRINT(" Z: ");
  WS_DEBUG_PRINTLNVAR(gz);

  // Report the magnitude of the magnetic field vector, in gauss.
  float magnitude_G = sqrtf(gx * gx + gy * gy + gz * gz);
  rawEvent->data[0] = magnitude_G;
  return true;
}
