/*!
 * @file WipperSnapper_I2C_Driver.h
 *
 * Base implementation for I2C device drivers.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef WipperSnapper_I2C_Driver_H
#define WipperSnapper_I2C_Driver_H

#include "Wippersnapper.h"
//extern Wippersnapper WS;

class WipperSnapper_I2C_Driver  {

public:
 // TODO: Initializer should use in wire _i2c?
  WipperSnapper_I2C_Driver(){
      // TODO
  }

  ~WipperSnapper_I2C_Driver() {
      // TODO
  }

  bool getInitialized() {
      return isInitialized;
  }

  void updateTemperature(float *temperature) {
      // base implementation
  }

  void updateHumidity(float *humidity) {
      // base implementation
  }

protected:
  bool isInitialized = false;
};

#endif // WipperSnapper_I2C_Driver_H