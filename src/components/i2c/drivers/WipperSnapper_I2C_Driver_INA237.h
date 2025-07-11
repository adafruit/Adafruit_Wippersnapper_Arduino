/*!
 * @file WipperSnapper_I2C_Driver_INA237.h
 *
 * Device driver for the INA237 DC Current and Voltage Monitor
 * 16-bit ADC with ±0.3% gain error, ±50µV offset voltage
 * Cost-effective version, lower precision than INA238
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
#ifndef WipperSnapper_I2C_Driver_INA237_H
#define WipperSnapper_I2C_Driver_INA237_H

#include "WipperSnapper_I2C_Driver.h"
#include "Wippersnapper.h"

// Forward declaration
class Adafruit_INA237;

/**************************************************************************/
/*!
    @brief  Class that provides a driver interface for a INA237 sensor.
*/
/**************************************************************************/
class WipperSnapper_I2C_Driver_INA237 : public WipperSnapper_I2C_Driver {
public:
  WipperSnapper_I2C_Driver_INA237(TwoWire *i2c, uint16_t sensorAddress);
  ~WipperSnapper_I2C_Driver_INA237();

  bool begin();
  bool getEventVoltage(sensors_event_t *voltageEvent);
  bool getEventCurrent(sensors_event_t *currentEvent);
  bool getEventRaw(sensors_event_t *powerEvent);

protected:
  Adafruit_INA237 *_ina237; ///< Pointer to INA237 sensor object
};

#endif // WipperSnapper_I2C_Driver_INA237