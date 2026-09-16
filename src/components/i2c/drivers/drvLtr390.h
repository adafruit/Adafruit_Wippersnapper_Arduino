/*!
 * @file drvLtr390.h
 *
 * Device driver for the LTR390 light sensor.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2023 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef DRV_LTR390_H
#define DRV_LTR390_H

#include "drvBase.h"
#include <Adafruit_LTR390.h>

#define LTR390_TICK_MS 50          ///< Step the ALS/UVS cycle every 50ms
#define LTR390_READ_LEAD_MS 1000   ///< Start the cycle 1s before a read is due
#define LTR390_MODE_SETTLE_MS 110  ///< Wait after a mode switch (100ms rate)
#define LTR390_DATA_TIMEOUT_MS 400 ///< Give up on a mode if no data by then

/*!
    @brief  Class that provides a driver interface for a LTR390 sensor.
*/
class drvLtr390 : public drvBase {
public:
  /*!
      @brief    Constructor for a LTR390 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvLtr390(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // The LTR390 measures either ALS or UVS, not both, and needs ~110ms after
    // a mode switch. fastTick() cycles ALS then UVS during the lead window
    // before each read is due, so the read pass never blocks.
    _fast_tick_ms = LTR390_TICK_MS;
    _tick_lead_ms = LTR390_READ_LEAD_MS;
  }

  /*!
      @brief    Destructor for an LTR390 sensor.
  */
  ~drvLtr390() { delete _ltr390; }

  /*!
      @brief    Initializes the LTR390 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _ltr390 = new Adafruit_LTR390();
    // Attempt to initialize LTR390
    if (!_ltr390->begin(_i2c))
      return false;

    // Configure LTR390 sensor
    // Note: This driver uses the default configuration from
    // https://github.com/adafruit/Adafruit_LTR390/blob/master/examples/ltr390_test/ltr390_test.ino
    _ltr390->setMode(LTR390_MODE_UVS);
    _ltr390->setGain(LTR390_GAIN_3);
    _ltr390->setResolution(LTR390_RESOLUTION_16BIT);
    return true;
  }

  /*!
      @brief    Puts the sensor in the given mode and starts the settle timer.
      @param    mode
                LTR390_MODE_ALS or LTR390_MODE_UVS.
  */
  void switchMode(ltr390_mode_t mode) {
    _ltr390->setMode(mode);
    _mode_start = millis();
  }

  /*!
      @brief    Background measurement step, called every LTR390_TICK_MS while
                a read is pending. Cycles ALS then UVS: switch mode, wait
                LTR390_MODE_SETTLE_MS, read once new data is flagged (or give
                up on that mode after LTR390_DATA_TIMEOUT_MS), then move on.
                When both modes have been visited the sample is filed with
                NewSample(); a metric whose mode produced no data is reported
                as unavailable by its accessor.
  */
  void fastTick() override {
    ulong now = millis();
    if (_first_tick || _phase == PHASE_DONE) {
      _have_als = false;
      _have_uvs = false;
      _phase = PHASE_ALS;
      switchMode(LTR390_MODE_ALS);
      return;
    }

    ulong waited = now - _mode_start;
    if (waited < LTR390_MODE_SETTLE_MS)
      return;

    bool got = _ltr390->newDataAvailable();
    if (got) {
      if (_phase == PHASE_ALS) {
        _als = _ltr390->readALS();
        _have_als = true;
      } else {
        _uvs = _ltr390->readUVS();
        _have_uvs = true;
      }
    } else if (waited < LTR390_DATA_TIMEOUT_MS) {
      return; // keep waiting for this mode
    }

    if (_phase == PHASE_ALS) {
      _phase = PHASE_UVS;
      switchMode(LTR390_MODE_UVS);
      return;
    }
    _phase = PHASE_DONE;
    if (_have_als || _have_uvs)
      NewSample();
  }

  /*!
      @brief    Gets the LTR390's ambient light reading, from the most recent
                cycle collected by fastTick().
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!AttemptRead() || !_have_als)
      return false;
    lightEvent->light = (float)_als;
    return true;
  }

  /*!
      @brief    Gets the LTR390's UV reading, from the most recent cycle
                collected by fastTick().
      @param    rawEvent
                Pointer to an adafruit sensor event.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!AttemptRead() || !_have_uvs)
      return false;
    rawEvent->data[0] = (float)_uvs;
    return true;
  }

protected:
  /*! Steps of the ALS/UVS measurement cycle driven by fastTick(). */
  enum Phase : uint8_t { PHASE_DONE, PHASE_ALS, PHASE_UVS };
  Adafruit_LTR390 *_ltr390;  ///< Pointer to LTR390 light sensor object
  Phase _phase = PHASE_DONE; ///< Current step of the measurement cycle
  ulong _mode_start = 0;     ///< millis() of the last mode switch
  uint32_t _als = 0;         ///< Last ALS reading
  uint32_t _uvs = 0;         ///< Last UVS reading
  bool _have_als = false;    ///< ALS was read in the current sample
  bool _have_uvs = false;    ///< UVS was read in the current sample
};

#endif // drvLtr390