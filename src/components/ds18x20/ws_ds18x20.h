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

#include <Adafruit_Sensor.h>
#include <DallasTemperature.h>

/** DS18x20 Object */
struct ds18x20Obj {
  OneWire *oneWire;
  DallasTemperature *dallasTempObj;
  DeviceAddress dallasTempAddr;
  wippersnapper_i2c_v1_I2CDeviceSensorProperties sensorPropeties[2];
};

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

  bool
  addDS18x20(wippersnapper_ds18x20_v1_Ds18x20InitRequest *msgDs18x20InitReq);
  void deleteDS18x20(
      wippersnapper_ds18x20_v1_Ds18x20DeInitRequest *msgDS18x20DeinitReq);
  void update();

private:
  std::vector<ds18x20Obj *> ds18xDrivers; ///< List of ds18x drivers
};
extern Wippersnapper WS;

#endif // WIPPERSNAPPER_DS18X20_H