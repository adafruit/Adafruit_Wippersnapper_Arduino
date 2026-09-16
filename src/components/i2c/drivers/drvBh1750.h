/*!
 * @file drvBh1750.h
 *
 * Device driver for a BH1750 Light sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Scott Perkins, 2022
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_1750_H
#define DRV_1750_H

#include "drvBase.h"
#include <hp_BH1750.h> //include the library for the BH1750 sensor

#define BH1750_TICK_MS 50        ///< Poll the conversion every 50ms
#define BH1750_READ_LEAD_MS 1000 ///< Start measuring 1s before a read

/*!
    @brief  Class that provides a driver interface for a BH1750 Light sensor.

            This driver uses the H-Resolution Mode and the default measurement
            time register (MTreg) of 69. According to the datasheet this is
            the recommended mode for most applications. Typical measurement
            time in this mode is 120ms

            This driver uses the One Time Measurement feature of the BH1750. The
            sensor returns to Power Down mode after each reading.
*/
class drvBh1750 : public drvBase {
public:
  /*!
      @brief    Constructor for a BH1750 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvBh1750(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // A high-resolution one-time measurement takes 120-180ms (datasheet
    // tHR), so it is started and collected by fastTick() in the lead window
    // before each read rather than blocking the read pass
    _fast_tick_ms = BH1750_TICK_MS;
    _tick_lead_ms = BH1750_READ_LEAD_MS;
  }

  /*!
      @brief    Destructor for a BH1750 sensor.
  */
  ~drvBh1750() {
    // Called when a BH1750 component is deleted.
    delete _bh1750;
  }

  /*!
      @brief  Initializes the BH1750 sensor and begins I2C.
              The set the quality to the H-Resolution Mode.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _bh1750 = new hp_BH1750();
    // attempt to initialize BH1750
    if (!_bh1750->begin(_address, _i2c))
      return false;
    // Set to the recommended quality setting
    _bh1750->setQuality(BH1750_QUALITY_HIGH);
    return true;
  }

  /*!
      @brief    Background measurement step, called every BH1750_TICK_MS
                while a read is pending: starts a one-time high-resolution
                measurement, then collects it once hasValue() reports it
                done. A saturated result (65535 counts) is not filed.
  */
  void fastTick() override {
    if (_first_tick || !_started) {
      _started = _bh1750->start();
      return;
    }
    if (!_bh1750->hasValue())
      return;
    _started = false;
    if (_bh1750->saturated())
      return;
    _lux = _bh1750->getLux();
    NewSample();
  }

  /*!
      @brief    Gets the ambient light reading collected by fastTick().
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!AttemptRead())
      return false;
    lightEvent->light = _lux;
    return true;
  }

protected:
  hp_BH1750 *_bh1750;    ///< Pointer to BH1750 light sensor object
  bool _started = false; ///< A one-time measurement is in flight
  float _lux = 0;        ///< Last collected reading, lux
};

#endif // drvBh1750