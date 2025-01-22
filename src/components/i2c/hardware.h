/*!
 * @file hardware.cpp
 *
 * Hardware driver for the i2c API
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
#ifndef WS_I2C_HARDWARE_H
#define WS_I2C_HARDWARE_H
#include "Wippersnapper_V2.h"

#ifdef ARDUINO_ARCH_RP2040
// Wire uses GPIO4 (SDA) and GPIO5 (SCL) automatically.
#define WIRE Wire
#endif

/**************************************************************************/
/*!
    @brief  Interface for I2c two-wire communication.
*/
/**************************************************************************/
class I2cHardware {
public:
  I2cHardware();
  ~I2cHardware();
  void InitDefaultBus();
  TwoWire *GetI2cBus() { return _i2c_bus; }
  wippersnapper_i2c_I2cBusStatus GetBusStatus() { return _bus_status; }
private:
  wippersnapper_i2c_I2cBusStatus _bus_status;
  void ToggleI2CPowerPin();
  TwoWire *_i2c_bus = nullptr;
};
#endif // WS_I2C_HARDWARE_H