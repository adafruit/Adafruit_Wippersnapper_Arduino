/*!
 * @file WipperSnapper_Component_I2C.h
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

// forward decl.
class Wippersnapper;

class WipperSnapper_Component_I2C {
public:
  WipperSnapper_Component_I2C(
      wippersnapper_i2c_v1_I2CInitRequest *msgInitRequest);
  ~WipperSnapper_Component_I2C();
  uint16_t scanAddresses(wippersnapper_i2c_v1_I2CScanRequest msgScanReq);

  int32_t _portNum;
  bool _isInit;
private:
  TwoWire *_i2c = NULL;
};
extern Wippersnapper WS;

#endif // WipperSnapper_Component_I2C_H