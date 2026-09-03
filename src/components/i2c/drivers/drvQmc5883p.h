/*!
 * @file drvQmc5883p.h
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
#ifndef DRV_QMC5883P_H
#define DRV_QMC5883P_H

#include "drvBase.h"

class Adafruit_QMC5883P; // forward

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a QMC5883P sensor.
*/
/**************************************************************************/
class drvQmc5883p : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a QMC5883P sensor.
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
  drvQmc5883p(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for a QMC5883P sensor.
  */
  /*******************************************************************************/
  ~drvQmc5883p();

  /*******************************************************************************/
  /*!
      @brief    Initializes the QMC5883P sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin();

  /*******************************************************************************/
  /*!
      @brief    Configures the QMC5883P sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override;

  /*******************************************************************************/
  /*!
      @brief    Applies the operating mode setting to the driver.
      @param    mode
                The mode index from the broker
                (0=Suspend, 1=Normal, 2=Single, 3=Continuous).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMode(const ws_config_Value &mode) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the output data rate (ODR) setting to the driver.
      @param    output_data_rate
                The output data rate index from the broker
                (0=10Hz, 1=50Hz, 2=100Hz, 3=200Hz).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setOutputDataRate(const ws_config_Value &output_data_rate) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the over-sample ratio (OSR) setting to the driver.
      @param    oversample_ratio
                The over-sample ratio index from the broker
                (0=8, 1=4, 2=2, 3=1).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setOverSampleRatio(const ws_config_Value &oversample_ratio) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the down-sample ratio (DSR) setting to the driver.
      @param    downsample_ratio
                The down-sample ratio index from the broker
                (0=1, 1=2, 2=4, 3=8).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setDownSampleRatio(const ws_config_Value &downsample_ratio) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the full-scale field range setting to the driver.
      @param    range
                The range index from the broker
                (0=±30 Gauss, 1=±12 Gauss, 2=±8 Gauss, 3=±2 Gauss).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setRange(const ws_config_Value &range) override;

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
  bool getEventRaw(sensors_event_t *rawEvent);

protected:
  Adafruit_QMC5883P *_qmc = nullptr; ///< Pointer to QMC5883P sensor object
};

#endif // DRV_QMC5883P_H
