/*!
 * @file drvIna238.h
 *
 * Device driver for the INA238 High Precision DC Current and Voltage Monitor
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
#ifndef DRV_INA238_H
#define DRV_INA238_H

#include "drvBase.h"

class Adafruit_INA238; ///< Forward declaration (lib header included in .cpp)

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA238 sensor.

    CUSTOM SETTINGS CANDIDATES (currently hard-coded in begin(); not yet
    exposed via the v2 properties API):
     - Shunt resistance & max expected current (setShunt) — calibration,
       depends on the physical shunt fitted to the board (0.015 ohm / 10 A
       default here).
     - ADC range (high vs low) for the shunt voltage.
     - Averaging count and bus/shunt conversion times.
*/
/**************************************************************************/
class drvIna238 : public drvBase {
public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for a INA238 sensor.
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
  drvIna238(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {}

  /*******************************************************************************/
  /*!
      @brief    Destructor for an INA238 sensor.
  */
  /*******************************************************************************/
  ~drvIna238();

  /*******************************************************************************/
  /*!
      @brief    Initializes the INA238 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin();

  /*******************************************************************************/
  /*!
      @brief    Configures the INA238 sensor with default settings.
      @returns  True if configured successfully, False otherwise.
  */
  /*******************************************************************************/
  bool configureDefaults() override;

  /*******************************************************************************/
  /*!
      @brief    Applies the shunt resistance (ohms) calibration value to the
                driver. Recalibrates using the stored max expected current.
      @param    shunt_resistance
                The shunt resistance, in ohms, from the broker (must be > 0).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setShuntResistance(const ws_config_Value &shunt_resistance) override;

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
  bool setMaxCurrent(const ws_config_Value &max_current) override;

  /*******************************************************************************/
  /*!
      @brief    Applies the ADC range setting to the driver (shunt-voltage
                range).
      @param    adc_range
                The ADC range index from the broker (0=high range, 1=low range).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setAdcRange(const ws_config_Value &adc_range) override;

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
                (0=50us, 1=84us, 2=150us, 3=280us, 4=540us, 5=1052us, 6=2074us,
                7=4120us).
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
                (0=50us, 1=84us, 2=150us, 3=280us, 4=540us, 5=1052us, 6=2074us,
                7=4120us).
      @returns  True if applied successfully, False otherwise.
  */
  /*******************************************************************************/
  bool setCurrentConversionTime(
      const ws_config_Value &current_conversion_time) override;

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
  Adafruit_INA238 *_ina238 = nullptr; ///< Pointer to INA238 sensor object
  float _shuntResistance = 0.015f;    ///< Shunt resistance, in ohms
  float _maxCurrent = 10.0f;          ///< Max expected current, in amps
};

#endif // DRV_INA238_H
