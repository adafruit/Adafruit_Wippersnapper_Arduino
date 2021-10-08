/*!
 * @file I2C_Driver.cpp
 *
 * Base class for a generic I2C sensor device driver.
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
#include "I2C_Driver.h"

/*******************************************************************************/
/*!
    @brief    Creates a generic I2C Driver object.
    @param    deviceAddress
              The i2c device's address.
    @param    i2c
              The desired i2c port.
*/
/*******************************************************************************/
I2C_Driver::I2C_Driver(uint16_t deviceAddress, TwoWire *i2c) {
  _i2c = i2c;
  _deviceAddr = deviceAddress;
  _pollPeriod = 0.0;
}

/*************************************/
/*!
    @brief    I2C driver destructor.
*/
/*************************************/
I2C_Driver::~I2C_Driver() {
  _i2c = NULL;
  _deviceAddr = 0;
  _pollPeriod = 0.0;
}

/*******************************************/
/*!
    @brief    Sets an I2C device's
                polling period, in seconds.
    @param    period
            The time elapsed between
            polling the I2C sensor for new
            data.
*/
/*******************************************/
void I2C_Driver::setPeriod(float period) { _pollPeriod = period; }

/* Sensor-Specific Funcs */

// AHTX0 Sensor //
/*************************************************/
/*!
    @brief  Initializes an AHTX0 temperature +
            humidity sensor.
    @return True if AHTX0 sensor initialized
            successfully, False otherwise.
*/
/*************************************************/
bool I2C_Driver::initAHTX0() {
  WS_DEBUG_PRINTLN("I2C_Driver::initAHTX0");
  bool is_success = true;
  _ahtx0 = new Adafruit_AHTX0();
  if (!_ahtx0->begin(_i2c)) {
    WS_DEBUG_PRINTLN("Error: AHTx0 not initialized");
    is_success = false;
  };
  WS_DEBUG_PRINTLN("AHT initialized successfully!");
  return is_success;
}

/***************************************************/
/*!
    @brief  Enables the AHTX0's temperature sensor.
*/
/***************************************************/
void I2C_Driver::enableAHTX0Temperature() {
  _ahtTemperature = _ahtx0->getTemperatureSensor();
  //_ahtTemperature->printSensorDetails();
}

/***************************************************/
/*!
    @brief  Enables the AHTX0's humidity sensor.
*/
/***************************************************/
void I2C_Driver::enableAHTX0Humidity() {
  _ahtHumidity = _ahtx0->getHumiditySensor();
  //_ahtHumidity->printSensorDetails();
}