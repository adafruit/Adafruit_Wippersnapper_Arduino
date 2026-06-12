/*!
 * @file WipperSnapper_I2C_Driver_INA238.h
 *
 * Device driver for the INA238 High-precision DC Current and Voltage Monitor
 * 16-bit ADC with ±0.1% gain error, ±5µV offset voltage
 * Higher precision version compared to INA237
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2025 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WipperSnapper_I2C_Driver_INA238_H
#define WipperSnapper_I2C_Driver_INA238_H

#include "WipperSnapper_I2C_Driver.h"

// Forward declaration
class Adafruit_INA238;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA238 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA238 : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_INA238(TwoWire *i2c, uint16_t sensorAddress);
  ~WipperSnapper_I2C_Driver_INA238();

  bool begin();
  bool getEventVoltage(sensors_event_t *voltageEvent);
  bool getEventCurrent(sensors_event_t *currentEvent);
  bool getEventRaw(sensors_event_t *powerEvent);

protected:
  Adafruit_INA238 *_ina238; ///< Pointer to INA238 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA238