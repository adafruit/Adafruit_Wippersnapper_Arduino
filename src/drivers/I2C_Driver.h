/*!
 * @file I2C_Driver.h
 *
 * Base class for a generic I2C sensor device driver.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef I2C_Driver_H
#define I2C_Driver_H

#include "Wippersnapper.h"

#include <Adafruit_AHTX0.h>

/**************************************************************************/
/*!
    @brief  Class that provides a sensor driver interface for the AHTX0.
*/
/**************************************************************************/
class I2C_Driver {
public:
  // GENERIC, shared
  I2C_Driver(uint16_t deviceAddress, TwoWire *i2c);
  ~I2C_Driver();
  void setPeriod(float period);

  // AHTX0-specific
  bool initAHTX0();
  void enableAHTX0Temperature();
  void enableAHTX0Humidity();

private:
  // General device driver properties
  int16_t _deviceAddr;
  float _pollPeriod;
  TwoWire *_i2c = NULL;
  // ATHX0
  Adafruit_AHTX0 *_ahtx0 = NULL;
  Adafruit_Sensor *_ahtTemperature = NULL;
  Adafruit_Sensor *_ahtHumidity = NULL;
};

#endif // I2C_Driver_H