/*!
 * @file WipperSnapper_I2C.h
 *
 * This component initiates I2C operations
 * using the Arduino generic TwoWire driver.
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
#ifndef WipperSnapper_Component_I2C_H
#define WipperSnapper_Component_I2C_H

#include "Wippersnapper.h"
#include <Wire.h>

#include "drivers/I2C_Driver.h"

// forward decl.
class Wippersnapper;
class I2C_Driver;

/**************************************************************************/
/*!
    @brief  Class that provides an interface with the I2C bus.
*/
/**************************************************************************/
class WipperSnapper_Component_I2C {
public:
  WipperSnapper_Component_I2C(
      wippersnapper_i2c_v1_I2CBusInitRequest *msgInitRequest);
  ~WipperSnapper_Component_I2C();
  wippersnapper_i2c_v1_I2CBusScanResponse scanAddresses();
  bool
  attachI2CDevice(wippersnapper_i2c_v1_I2CDeviceInitRequest *msgDeviceInitReq);
  bool isInitialized();

private:
  bool _isInit;
  int32_t _portNum;
  TwoWire *_i2c = NULL;
  std::vector<I2C_Driver *> activeDrivers;
};
extern Wippersnapper WS;

#endif // WipperSnapper_Component_I2C_H