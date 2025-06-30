/*!
 * @file WipperSnapper_I2C_Driver_INA260.cpp
 *
 * Device driver implementation for the INA260 DC Current and Voltage Monitor
 *
 */

#include "WipperSnapper_I2C_Driver_INA260.h"
#include "../../../Wippersnapper.h"
#include <Adafruit_INA260.h>

/*******************************************************************************/
/*!
    @brief    Constructor for a INA260 sensor.
    @param    i2c
              The I2C interface.
    @param    sensorAddress
              The 7-bit I2C address of the sensor.
*/
/*******************************************************************************/
WipperSnapper_I2C_Driver_INA260::WipperSnapper_I2C_Driver_INA260(TwoWire *i2c, uint16_t sensorAddress)
    : WipperSnapper_I2C_Driver(i2c, sensorAddress), _ina260(nullptr) {
  _i2c = i2c;
  _sensorAddress = sensorAddress;
}

/*******************************************************************************/
/*!
    @brief    Destructor for an INA260 sensor.
*/
/*******************************************************************************/
WipperSnapper_I2C_Driver_INA260::~WipperSnapper_I2C_Driver_INA260() {
  delete _ina260;
}

/*******************************************************************************/
/*!
    @brief    Initializes the INA260 sensor and begins I2C.
    @returns  True if initialized successfully, False otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_I2C_Driver_INA260::begin() {
  _ina260 = new Adafruit_INA260();
  if (!_ina260->begin(_sensorAddress, _i2c)) {
    WS_DEBUG_PRINTLN("INA260 failed to initialise!");
    return false;
  }
  // TODO: use setCalibration()

  return true;
}

/*******************************************************************************/
/*!
    @brief    Reads a voltage sensor and converts the
              reading into the expected SI unit.
    @param    voltageEvent
              voltage sensor reading, in volts.
    @returns  True if the sensor event was obtained successfully, False
              otherwise.
*/
/*******************************************************************************/
bool WipperSnapper_I2C_Driver_INA260::getEventVoltage(sensors_event_t *voltageEvent) {
  voltageEvent->voltage = _ina260->readBusVoltage();
  return true;
}

/**
 * @brief   Get the current sensor event.
 *
 * @param   currentEvent  Pointer to the current sensor event.
 *
 * @returns True if the sensor event was obtained successfully, False
 * otherwise.
 */
bool WipperSnapper_I2C_Driver_INA260::getEventCurrent(sensors_event_t *currentEvent) {
  currentEvent->current = _ina260->readCurrent();
  return true;
}