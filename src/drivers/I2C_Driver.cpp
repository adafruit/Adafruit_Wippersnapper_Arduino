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
    @param    _i2c
              The desired i2c port.
*/
/*******************************************************************************/
I2C_Driver::I2C_Driver(uint16_t deviceAddress, TwoWire *i2c) {
    WS_DEBUG_PRINTLN("* New I2CDriver!");
    WS_DEBUG_PRINT("* I2CDriver Device Addr: ");WS_DEBUG_PRINTLN(deviceAddress);
    _i2c = i2c;
    _pollPeriod = 0.0;
}

/*************************************/
/*!
    @brief    I2C driver destructor.
*/
/*************************************/
I2C_Driver::~I2C_Driver() {
    _i2c = NULL;
    _pollPeriod = 0.0;
}

/*******************************************/
/*!
    @brief    Sets an I2C device's
                polling period, in seconds.
*/
/*******************************************/
void I2C_Driver::setPeriod(float period) {
  _pollPeriod = period;
}

/*************************************************/
/*!
    @brief  Initializes an I2C driver. Must
    be implemented in an I2C_Driver_x class.
    @return True if I2C device driver initialized
            successfully, False otherwise.
*/
/*************************************************/
bool I2C_Driver::initSensor() {
    WS_DEBUG("I2C_Driver initSensor()");
    return true;
}

/*************************************************/
/*!
    @brief  Polls an I2C driver for new sensor
            messages. Must be implemented in an
            I2C_Driver_x class.
    @return True if I2C device driver initialized
            successfully, False otherwise.
*/
/*************************************************/
void I2C_Driver::pollSensor() {
  WS_DEBUG("I2C_Driver PollSensor()");
}

// Sensor-Specific, AHTX0
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

void I2C_Driver::enableAHTX0Temperature() {
  _ahtTemperature = _ahtx0->getTemperatureSensor();
  _ahtTemperature->printSensorDetails();
  WS_DEBUG_PRINTLN("Enabled AHTX0 temperature sensor");
}