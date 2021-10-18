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
    _aht_temp_period = 0.0;
    _aht_humidity_period = 0.0;
  }

  ~WipperSnapper_I2C_Driver_AHTX0() {
    _aht_temp = NULL;
    _aht_temp_period = 0.0;
    _aht_humidity = NULL;
    _aht_humidity_period = 0.0;
  }

  void enableTemperatureSensor() {
    _aht_temp = _aht.getTemperatureSensor();
  }

  void enableHumiditySensor() {
    _aht_humidity = _aht.getHumiditySensor();
  }

  void disableTemperatureSensor() {
    _aht_temp = NULL;
  }

  void disableHumiditySensor() {
    _aht_humidity = NULL;
  }

  void setTemperatureSensorPeriod(float tempPeriod) {
    _aht_temp_period = tempPeriod;
  }

  void setHumiditySensorPeriod(float humidPeriod) {
    _aht_humidity_period = humidPeriod;
  }

  float getTemperatureSensorPeriod() {
    return _aht_temp_period;
  }

  float getHumiditySensorPeriod() {
    return _aht_humidity_period;
  }

  void updateTemperature(float *temperature) {
    sensors_event_t temp;
    // update temp, if sensor enabled
    if (_aht_temp != NULL) {
      _aht_temp->getEvent(&temp);
      *temperature = temp.temperature;
    }
  }

  void updateHumidity(float *humidity) {
    sensors_event_t humid;
    // update humid, if sensor enabled
    if (_aht_humidity != NULL) {
      _aht_humidity->getEvent(&humid);
      *humidity = humid.relative_humidity;
    }
  }

protected:
  Adafruit_AHTX0 _aht;
  Adafruit_Sensor *_aht_temp, *_aht_humidity = NULL;
  float _aht_temp_period, _aht_humidity_period;
};

#endif // WipperSnapper_I2C_Driver_AHTX0