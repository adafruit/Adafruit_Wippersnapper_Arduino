/*!
 * @file drvIna260.h
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
#ifndef DRV_INA260_H
#define DRV_INA260_H

#include "drvBase.h"

class Adafruit_INA260; ///< Forward declaration (lib header included in .cpp)

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA260 sensor.

    CUSTOM SETTINGS CANDIDATES (currently hard-coded in begin(); not yet
    exposed via the v2 properties API):
     - Averaging count (setAveragingCount): trades noise for update rate.
     - Bus/shunt voltage conversion times (set*ConversionTime).
     - Operating / triggered measurement mode.
    The INA260 has an integrated shunt, so no shunt calibration is required.
*/
/**************************************************************************/
class drvIna260 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA260 sensor.
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
  drvIna260(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA260 sensor.
  */
  /*******************************************************************************/
  ~drvIna260();

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA260 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin();

  /*******************************************************************************/
  /*!
      @brief    Configures the INA260 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override;

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
  bool setAveragedSamples(const ws_config_Value &averaged_samples) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the bus voltage ADC conversion time setting to the
                driver.
      @param    voltage_conversion_time
                The voltage conversion time index from the broker
                (0=140us, 1=204us, 2=332us, 3=588us, 4=1.1ms, 5=2.116ms,
                6=4.156ms, 7=8.244ms).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setVoltageConversionTime(
      const ws_config_Value &voltage_conversion_time) override;

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
  bool setCurrentConversionTime(
      const ws_config_Value &current_conversion_time) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the measurement mode setting to the driver.
      @param    mode
                The measurement mode index from the broker
                (0=Shutdown, 1=Triggered, 2=Continuous).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setMode(const ws_config_Value &mode) override;

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
  bool getEventVoltage(sensors_event_t *voltageEvent);

  /**
   * @brief   Get the current sensor event.
   *
   * @param   currentEvent  Pointer to the current sensor event.
   *
   * @returns True if the sensor event was obtained successfully, False
   * otherwise.
   */
  bool getEventCurrent(sensors_event_t *currentEvent);

protected:
  Adafruit_INA260 *_ina260 = nullptr; ///< Pointer to INA260 sensor object
};

#endif // DRV_INA260_H
