/*!
 * @file I2C_Driver_AHTX0.cpp
 *
 * Subclass for an AHT10 & AHT20 Humidity and Temperature Sensor.
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
#include "I2C_Driver_AHTX0.h"


/*************************************************/
/*!
    @brief  Initializes an AHTX0 sensor.
    @return True if AHTX0 device driver initialized
            successfully, False otherwise.
*/
/*************************************************/
bool I2C_Driver_AHTX0::initSensor() {
  bool is_success = true; 
  _ahtx0 = new Adafruit_AHTX0();
  if (!_ahtx0->begin(_i2c)) {
      WS_DEBUG_PRINTLN("Error: AHTx0 not initialized");
      is_success = false;
  };
  WS_DEBUG_PRINTLN("AHT initialized successfully!");
  return is_success;
}

/*************************************************/
/*!
    @brief  Polls the AHTX0 for new data.
*/
/*************************************************/
void I2C_Driver_AHTX0::pollSensor() {
  // TODO - Validate sensors and poll them
}

/**************************************************/
/*!
    @brief  Enables the AHTX0's temperature sensor.
*/
/**************************************************/
void I2C_Driver_AHTX0::enableSensorTemperature() {
  _ahtTemperature = _ahtx0->getTemperatureSensor();
  WS_DEBUG_PRINTLN("Enabled AHTX0 temperature sensor");
}

/**************************************************/
/*!
    @brief  Enables the AHTX0's humidity sensor.
*/
/**************************************************/
void I2C_Driver_AHTX0::enableSensorHumidity() {
  _ahtHumidity = _ahtx0->getHumiditySensor();
  WS_DEBUG_PRINTLN("Enabled AHTX0 humidity sensor");
}