/*!
 * @file drvStemmaSoil.h
 *
 * Device driver for the STEMMA Soil Sensor
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Marcus Wu 2022
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */

#ifndef DRV_STEMMA_SOIL_H
#define DRV_STEMMA_SOIL_H

#include "drvBase.h"

// Only include Adafruit_seesaw.h from drvStemmaSoil.cpp: it #defines
// INPUT_PULLDOWN as 0x03 when the core has no such macro (e.g. 0x03 is
// OUTPUT_OPEN_DRAIN on ESP8266), which would leak into every file that
// includes this header.
class Adafruit_seesaw;

/*!
    @brief  Class that provides a driver interface for the STEMMA soil sensor.
*/
class drvStemmaSoil : public drvBase {
public:
  drvStemmaSoil(TwoWire *i2c, uint16_t sensorAddress, uint32_t mux_channel,
                const char *driver_name);

  ~drvStemmaSoil();
  bool begin() override;
  bool getEventAmbientTemp(sensors_event_t *tempEvent);
  bool getEventRaw(sensors_event_t *rawEvent);

protected:
  Adafruit_seesaw *_seesaw = nullptr; ///< Seesaw object
};

#endif // DRV_STEMMA_SOIL_H