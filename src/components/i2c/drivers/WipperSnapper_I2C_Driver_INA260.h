/*!
 * @file WipperSnapper_I2C_Driver_INA260.h
 *
 * Device driver for the INA260 DC Current and Voltage Monitor
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
#ifndef WipperSnapper_I2C_Driver_INA260_H
#define WipperSnapper_I2C_Driver_INA260_H

#include "WipperSnapper_I2C_Driver.h"

// Forward declaration
class Adafruit_INA260;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA260 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA260 : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_INA260(TwoWire *i2c, uint16_t sensorAddress);
  ~WipperSnapper_I2C_Driver_INA260();

  bool begin();
  bool getEventVoltage(sensors_event_t *voltageEvent);
  bool getEventCurrent(sensors_event_t *currentEvent);

protected:
  Adafruit_INA260 *_ina260 = nullptr; ///< Pointer to INA260 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA260