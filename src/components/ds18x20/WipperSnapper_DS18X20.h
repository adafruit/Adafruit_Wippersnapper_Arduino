/*!
 * @file WipperSnapper_DS18X20.h
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

#ifdef ARDUINO_ARCH_ESP32
// PaulStoffregen OneWire is incompatable with arduino-esp32 v2.0.1
// see: https://github.com/PaulStoffregen/OneWire/issues/112 and
// https://github.com/espressif/arduino-esp32/issues
#include <OneWireNg.h>
#elif
// Non-arduino-esp32 arch. should use PaulStoffregen OneWire
#include <OneWire.h>
#endif

#include <Adafruit_Sensor.h>
#include <DallasTemperature.h>

// forward decl.
class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Class that provides an interface with DS18X20-compatible
            sensors.
*/
/**************************************************************************/
class WipperSnapper_DS18X20 {
public:
  WipperSnapper_DS18X20(
      wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq);
  ~WipperSnapper_DS18X20();

  bool begin();

  uint8_t getResolution();
  uint8_t *getAddress();
  int32_t getPin();
  void update();

private:
  OneWire *_wire;               /*!< Pointer to OneWire instance. */
  DallasTemperature *_ds;       /*!< Pointer to DallasTemperature instance. */
  DeviceAddress _sensorAddress; /*!< Sensor's unique 64-bit identifier. */
  int32_t _sensorPin;           /*!< Pin used for OneWire bus. */
  uint8_t _resolution;   /*!< Sensor resolution (9, 10, 11, or 12 bits). */
  long _sensorPeriod;    /*!< Time between sensor reads, in milliseconds. */
  long _sensorPeriodPrv; /*!< Last time the sensor was read, in milliseconds. */
};

extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DS18X20_H