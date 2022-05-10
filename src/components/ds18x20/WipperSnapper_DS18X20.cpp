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
    // TODO

    // Set sensor resolution
    _resolution = msgDs18x20InitReq->sensor_resolution;
    // Set sensor period
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

int32_t WipperSnapper_DS18X20::getPin() {
    return _sensorPin;
}

/*************************************************************/
/*!
    @brief    TODO for a WipperSnapper DS18X20 component.
*/
/*************************************************************/
void WipperSnapper_DS18X20::update() {
  // TODO
}


