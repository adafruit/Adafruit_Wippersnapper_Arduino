/*!
 * @file model.h
 *
 * Provides high-level interfaces for messages within i2c.proto.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_I2C_MODEL_H
#define WS_I2C_MODEL_H
#include "Wippersnapper_V2.h"

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from i2c.proto.
*/
/**************************************************************************/
class I2cModel {
public:
  I2cModel();
  ~I2cModel();
  // I2cBusScan
  // I2cDeviceAddOrReplace
  // I2cDeviceRemove
private:
  // I2C sub-messages go here
};
#endif // WS_I2C_MODEL_H