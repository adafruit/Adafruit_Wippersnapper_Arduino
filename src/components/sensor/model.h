/*!
 * @file src/components/sensor/model.h
 *
 * Model for the sensor.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SENSOR_MODEL_H
#define WS_SENSOR_MODEL_H
#include "protos/sensor.pb.h"
#include "wippersnapper.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from sensor.proto.
*/
class SensorModel {
public:
  SensorModel();
  ~SensorModel();

private:
  ws_sensor_Event _msg_sensor_event; ///< SensorEvent message
};
#endif // WS_SENSOR_MODEL_H