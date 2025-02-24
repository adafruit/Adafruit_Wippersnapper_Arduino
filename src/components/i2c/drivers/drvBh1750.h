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

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a BH1750 Light sensor.

            This driver uses the H-Resolution Mode and the default measurement
            time register (MTreg) of 69. According to the datasheet this is
            the recommended mode for most applications. Typical measurement
            time in this mode is 120ms

            This driver uses the One Time Measurement feature of the BH1750. The
            sensor returns to Power Down mode after each reading.
*/
/**************************************************************************/
class drvBh1750 : public drvBase {
public:
  /*******************************************************************************/
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
  /*******************************************************************************/
  drvBh1750(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
            const char *driver_name)
      : drvBase(i2c, sensorAddress, mux_channel, driver_name) {
    // Initialization handled by drvBase constructor
  }

  /*******************************************************************************/
  /*!
      @brief    Destructor for a BH1750 sensor.
  */
  /*******************************************************************************/
  ~drvBh1750() {
    // Called when a BH1750 component is deleted.
    delete _bh1750;
  }

  /*******************************************************************************/
  /*!
      @brief  Initializes the BH1750 sensor and begins I2C.
              The set the quality to the H-Resolution Mode.
      @returns  True if initialized successfully, False otherwise.
  */
  /*******************************************************************************/
  bool begin() override {
    _bh1750 = new hp_BH1750();
    // attempt to initialize BH1750
    if (!_bh1750->begin(_address, _i2c))
      return false;
    // Set to the recommended quality setting
    _bh1750->setQuality(BH1750_QUALITY_HIGH);
    return true;
  }

  /*******************************************************************************/
  /*!
      @brief    Performs a light sensor read using the One Time Measurement
                feature of the BH1750. The sensor goes to Power Down mode after
                each reading.
      @param    lightEvent
                Light sensor reading, in lux.
      @returns  True if the sensor event was obtained successfully, False
                otherwise.
  */
  /*******************************************************************************/
  bool getEventLight(sensors_event_t *lightEvent) {
    _bh1750->start();
    lightEvent->light = _bh1750->getLux();
    return true;
  }

protected:
  hp_BH1750 *_bh1750; ///< Pointer to BH1750 light sensor object
};

#endif // drvBh1750