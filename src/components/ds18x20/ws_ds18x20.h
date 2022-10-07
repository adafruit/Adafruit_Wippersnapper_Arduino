/*!
 * @file ws_ds18x20.h
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
#ifndef WIPPERSNAPPER_DS18X20_H
#define WIPPERSNAPPER_DS18X20_H

#include "Wippersnapper.h"

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that provides an interface with DS18X20-compatible
            sensors.
*/
/**************************************************************************/
class ws_ds18x20 {
public:
  ws_ds18x20();
  ~ws_ds18x20();

  bool addDS18x20(wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq);
  void deleteDS18x20(wippersnapper_ds18x20_v1_Ds18x20DeInitRequest *msgDS18x20DeinitReq);
  void update();

  // TODO: These need to get moved to DRIVER code
  // note: maybe add() instead of begin?
  //bool begin(wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq);
  //uint8_t getResolution();
  //uint8_t *getAddress();
  //int32_t getPin();

private:
//  OneWire *_wire;               /*!< Pointer to OneWire instance. */
//  DallasTemperature *_ds;       /*!< Pointer to DallasTemperature instance. */
//  DeviceAddress _sensorAddress; /*!< Sensor's unique 64-bit identifier. */
//  int32_t _sensorPin;           /*!< Pin used for OneWire bus. */
//  uint8_t _resolution;   /*!< Sensor resolution (9, 10, 11, or 12 bits). */
//  long _sensorPeriod;    /*!< Time between sensor reads, in milliseconds. */
//  long _sensorPeriodPrv; /*!< Last time the sensor was read, in milliseconds. */
};

extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DS18X20_H