/*!
 * @file model.h
 *
 * Model for the sensor.proto message.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SENSOR_MODEL_H
#define WS_SENSOR_MODEL_H
#include "Wippersnapper_V2.h"
#include "protos/sensor.pb.h"

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from sensor.proto.
*/
/**************************************************************************/
class SensorModel {
public:
  SensorModel();
  ~SensorModel();
  bool EncodeSensorEventMessage();
  wippersnapper_sensor_SensorEvent *GetSensorEventMessage();
  // These functions create a SensorEvent message
  void CreateSensorEventFloat(wippersnapper_sensor_SensorType sensor_type, float sensor_value);
private:
  wippersnapper_sensor_SensorEvent _msg_sensor_event; ///< SensorEvent message
};
#endif // WS_SENSOR_MODEL_H