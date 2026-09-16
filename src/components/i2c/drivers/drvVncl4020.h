/*!
 * @file drvVncl4020.h
 *
 * Device driver for the VCNL4020 light + proximity sensor.
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
#ifndef DRV_VNCL4020_H
#define DRV_VNCL4020_H

#include "drvBase.h"
#include <Adafruit_VCNL4020.h>

/*!
    @brief  Class that provides a driver interface for a VCNL4020 sensor.
*/
class drvVncl4020 : public drvBase {
public:
  /*!
      @brief    Constructor for a VCNL4020 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                The 7-bit I2C address of the sensor.
      @param    mux_channel
                The I2C multiplexer channel.
      @param    driver_name
                The name of the driver.
  */
  drvVncl4020(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
              const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*!
      @brief    Destructor for an VCNL4020 sensor.
  */
  ~drvVncl4020() { delete _vcnl4020; }

  /*!
      @brief    Initializes the VCNL4020 sensor and begins I2C.
      @returns  True if initialized successfully, False otherwise.
  */
  bool begin() override {
    _vcnl4020 = new Adafruit_VCNL4020();
    // Attempt to initialize and configure VCNL4020
    return _vcnl4020->begin(_i2c, _address);
  }

  /*!
      @brief    Latches the VCNL4020's ambient and proximity data-ready flags
                for the pass (Command register als_data_rdy / prox_data_rdy;
                each clears when its result register is read).
      @returns  True if either channel has a new result, False otherwise.
  */
  bool IsSensorReady() override {
    _als_ready = _vcnl4020->isAmbientReady();
    _prox_ready = _vcnl4020->isProxReady();
    return _als_ready || _prox_ready;
  }

  /*!
      @brief    Gets the ambient light reading, in lux.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  bool getEventLight(sensors_event_t *lightEvent) {
    if (!AttemptRead() || !_als_ready)
      return false;
    // Datasheet: ambient light resolution 0.25 lx per count
    lightEvent->light = 0.25f * (float)_vcnl4020->readAmbient();
    return true;
  }

  /*!
      @brief    Reads the VCNL4020's proximity value into an event (no unit).
      @param    proximityEvent
                Pointer to an Adafruit_Sensor event.
      @returns  True if the proximity was obtained successfully, False
                otherwise.
  */
  bool getEventProximity(sensors_event_t *proximityEvent) {
    if (!AttemptRead() || !_prox_ready)
      return false;
    proximityEvent->data[0] = (float)_vcnl4020->readProximity();
    return true;
  }

protected:
  Adafruit_VCNL4020 *_vcnl4020; ///< Pointer to VCNL4020 light sensor object
  bool _als_ready = false;      ///< Ambient result ready this pass
  bool _prox_ready = false;     ///< Proximity result ready this pass
};

#endif // drvVncl4020