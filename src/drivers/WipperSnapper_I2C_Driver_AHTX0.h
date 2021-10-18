/*!
 * @file WipperSnapper_I2C_Driver_AHTX0.h
 *
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

#ifndef WipperSnapper_I2C_Driver_AHTX0_H
#define WipperSnapper_I2C_Driver_AHTX0_H

#include "Wippersnapper.h"
#include "WipperSnapper_I2C_Driver.h"
#include <Adafruit_AHTX0.h>
//extern Wippersnapper WS;

class WipperSnapper_I2C_Driver_AHTX0 : public WipperSnapper_I2C_Driver {

public:
  WipperSnapper_I2C_Driver_AHTX0(TwoWire *_i2c) : WipperSnapper_I2C_Driver() {
    isInitialized = _aht.begin(_i2c);
  }

  ~WipperSnapper_I2C_Driver_AHTX0() {
      // TODO
  }

  void enable_aht_temperature_sensor() {
    _aht_temp = _aht.getTemperatureSensor();
  }

  void enable_aht_humidity_sensor() {
    _aht_humidity = _aht.getHumiditySensor();
  }

  void update_aht20(float *temperature, float *humidity) {
    // update temp, if sensor enabled
    if (_aht_temp != NULL) {
      _aht_temp->getEvent(&_temp);
      *temperature = _temp.temperature;
    }

    // update humid, if sensor enabled
    if (_aht_humidity != NULL) {
      _aht_humidity->getEvent(&_humidity);
      *humidity = _humidity.relative_humidity;
    }
  }

protected:
  Adafruit_AHTX0 _aht;
  Adafruit_Sensor *_aht_temp, *_aht_humidity = NULL;
  sensors_event_t _temp, _humidity;
};

#endif // WipperSnapper_I2C_Driver_AHTX0