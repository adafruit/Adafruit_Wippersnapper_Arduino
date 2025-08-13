/*!
 * @file WipperSnapper_I2C_Driver_INA228.h
 *
 * Device driver for the INA228 High Precision DC Current and Voltage Monitor
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
#ifndef WipperSnapper_I2C_Driver_INA228_H
#define WipperSnapper_I2C_Driver_INA228_H

#include "WipperSnapper_I2C_Driver.h"
#include "Wippersnapper.h"

// Forward declaration
class Adafruit_INA228;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA228 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA228 : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_INA228(TwoWire *i2c, uint16_t sensorAddress);
  ~WipperSnapper_I2C_Driver_INA228();

  bool begin();
  bool getEventVoltage(sensors_event_t *voltageEvent);
  bool getEventCurrent(sensors_event_t *currentEvent);
  bool getEventRaw(sensors_event_t *powerEvent);

protected:
  Adafruit_INA228 *_ina228; ///< Pointer to INA228 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA228_H
