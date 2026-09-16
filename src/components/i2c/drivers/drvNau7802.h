/*!
 * @file drvNau7802.h
 *
 * Device driver for the NAU7802 24bit ADC / load cell breakout
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry for Adafruit Industries 2024
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_NAU7802_H
#define DRV_NAU7802_H

#include "drvBase.h"
#include <Adafruit_NAU7802.h>

#define NAU7802_TICK_MS 50       ///< Poll for a new ADC sample every 50ms
#define NAU7802_READ_LEAD_MS 500 ///< Start polling 0.5s before a read is due

/*!
    @brief  Class that provides a driver interface for the NAU7802.
*/
class drvNau7802 : public drvBase {
public:
  /*!
      @brief    Constructor for an NAU7802.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvNau7802(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
             const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // The ADC runs at 10 SPS; rather than spin-wait for available() in the
    // read pass, samples are harvested by fastTick() during the lead window
    // before each read is due.
    _fast_tick_ms = NAU7802_TICK_MS;
    _tick_lead_ms = NAU7802_READ_LEAD_MS;
  }

  /*!
      @brief    Destructor for an NAU7802.
  */
  ~drvNau7802() { _nau7802 = nullptr; }

  /*!
      @brief    Initializes the NAU7802 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override { return _nau7802->begin(_i2c) && configure_nau7802(); }

  /*!
      @brief    Configures the NAU7802 sensor.
      @returns  True if configured successfully, False otherwise.
  */
  bool configure_nau7802() {
    if (!_nau7802->setLDO(NAU7802_3V0)) {
      // WS_DEBUG_PRINTLN("Failed to set LDO to 3V0");
      return false;
    }

    if (!_nau7802->setGain(NAU7802_GAIN_128)) {
      // WS_DEBUG_PRINTLN("Failed to set gain to 128");
      return false;
    }

    if (!_nau7802->setRate(NAU7802_RATE_10SPS) &&
        !_nau7802->setRate(NAU7802_RATE_10SPS)) {
      // WS_DEBUG_PRINTLN("Failed to set sample rate to 10SPS");
      return false;
    }

    // Take 10 readings to flush out old readings (10 samples per second)
    flushNAU7802(10);

    for (int retries = 0; retries < 3; retries++) {
      if (_nau7802->calibrate(NAU7802_CALMOD_INTERNAL)) {
        // WS_DEBUG_PRINTLN("Calibrated internal offset");
        return true;
      }
      // WS_DEBUG_PRINTLN("Failed to calibrate internal offset, retrying!");
      delay(ONE_SECOND_IN_MS);
    }
    // WS_DEBUG_PRINTLN("ERROR: Failed to calibrate internal offset of
    // NAU7802.");
    return false;
  }

  /*!
      @brief    Gets datapoints from sensor and discards (flushes old data).
      @param    count
                Number of readings to discard.
  */
  void flushNAU7802(uint8_t count) {
    for (uint8_t skipCounter = 0; skipCounter < count; skipCounter++) {
      while (!_nau7802->available())
        delay(1);
      _nau7802->read();
    }
  }

  /*!
      @brief    Background sampling step, called every NAU7802_TICK_MS while a
                read is pending: takes the ADC sample if one is available and
                files it with NewSample().
  */
  void fastTick() override {
    if (!_nau7802->available())
      return;
    _raw = _nau7802->read();
    NewSample();
  }

  /*!
      @brief    Gets the sensor's raw "force" value, from the most recent
                sample collected by fastTick().
      @param    rawEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the reading was obtained successfully, False otherwise.
  */
  bool getEventRaw(sensors_event_t *rawEvent) {
    if (!AttemptRead())
      return false;
    rawEvent->data[0] = (float)_raw;
    return true;
  }

protected:
  Adafruit_NAU7802 *_nau7802 = nullptr; ///< NAU7802 object
  int32_t _raw = 0;                     ///< Last ADC sample
};

#endif // DRV_NAU7802_H