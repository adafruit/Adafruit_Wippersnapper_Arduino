/*!
 * @file WipperSnapper_DS18X20.cpp
 *
 * This component implements 1-wire communication
 * for the DS18X20-line of Maxim Temperature ICs.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "WipperSnapper_DS18X20.h"

/*************************************************************/
/*!
    @brief    Creates a new WipperSnapper Ds18x20 component.
    @param    msgInitRequest
              The Ds18x20 initialization request message.
*/
/*************************************************************/
WipperSnapper_DS18X20::WipperSnapper_DS18X20(wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq) {
    // Set sensor pin
    _sensorPin = msgDs18x20InitReq->onewire_pin;
    // Initialize OneWire instance
    _wire = new OneWire();

    // Initialize DallasTemperature instance
    _ds = new DallasTemperature(_wire);

    // Set sensor properties
    _resolution = msgDs18x20InitReq->sensor_resolution;
    // TODO: May want to look at this
    _sensorPeriod = msgDs18x20InitReq->sensor_period * 1000;
}

/*************************************************************/
/*!
    @brief    Destructor for a WipperSnapper DS18X20 component.
*/
/*************************************************************/
WipperSnapper_DS18X20::~WipperSnapper_DS18X20() {
  // TODO
}

bool WipperSnapper_DS18X20::begin() {
  // Attempt to get address from DS sensor at index 0
  if (!_ds->getAddress(_sensorAddress, 0))
    return false;

  // Check if address is within the family of sensors the Arduino-Temperature-Control-Library supports
  if (!_ds->validFamily(_sensorAddress))
    return false;

  // Attempt to set DS sensor's resolution
  _ds->setResolution(_sensorAddress, _resolution);

  return true;
}

int32_t WipperSnapper_DS18X20::getPin() {
    return _sensorPin;
}

uint8_t* WipperSnapper_DS18X20::getAddress() {
  _ds->getAddress(_sensorAddress, 0);
  return _sensorAddress;
}

uint8_t WipperSnapper_DS18X20::getResolution() {
  return _ds->getResolution(_sensorAddress);
}

/*************************************************************/
/*!
    @brief    TODO for a WipperSnapper DS18X20 component.
*/
/*************************************************************/
void WipperSnapper_DS18X20::update() {
  // TODO
}


