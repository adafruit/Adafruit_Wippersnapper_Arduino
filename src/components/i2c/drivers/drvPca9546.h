/*!
 * @file drvPca9546.h
 *
 * Device driver for a PCA9546 4-channel I2C MUX.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_PCA_9546_H
#define DRV_PCA_9546_H

#include "drvBase.h"

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver for a PCA9546 I2C Mux.
*/
/**************************************************************************/
class drvPca9546 : public drvBase {

public:
  /*******************************************************************************/
  /*!
      @brief    Constructor for an BME280 sensor.
      @param    i2c
                The I2C interface.
      @param    sensorAddress
                7-bit device address.
      @param    mux_channel
                The I2C MUX channel.
  */
  /*******************************************************************************/
  drvPca9546(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel)
      : drvBase(i2c, sensorAddress, mux_channel) {
    _i2c = i2c;
    _address = sensorAddress;
    _i2c_mux_channel = mux_channel; // Eh, we probably don't need this in this driver, TODO!
  }

  bool begin() override {
    // Do nothing
    return true;
  }

  void SelectMUXChannel(uint8_t channel) {
    if (channel > 3) return;
    _i2c->beginTransmission(_address);
    _i2c->write(channel << channel);
    _i2c->endTransmission();
  }
protected:

};

#endif // drvPca9546