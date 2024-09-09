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

class SensorModel {
public:
  SensorModel();
  ~SensorModel();
  bool decodeSensorEvent(pb_istream_t *stream);
  void clearSensorEvent();
  wippersnapper_sensor_SensorEvent *getSensorEvent() {
    return &_msg_sensor_event;
  }
  wippersnapper_sensor_SensorType getSensorType() {
    return _msg_sensor_event.type;
  } ///< Returns sensor's type and corresponding SI unit
  pb_size_t getWhichValue() {
    return _msg_sensor_event.which_value;
  } ///< Returns the value union's tag
  float getValueFloat() { return _msg_sensor_event.value.float_value; }
  bool getValueBool() { return _msg_sensor_event.value.bool_value; }
  wippersnapper_sensor_SensorEvent_SensorEvent3DVector getValueVector() {
    return _msg_sensor_event.value.vector_value;
  }
  wippersnapper_sensor_SensorEvent_SensorEventOrientation
  getValueOrientation() {
    return _msg_sensor_event.value.orientation_value;
  }
  wippersnapper_sensor_SensorEvent_SensorEventColor getValueColor() {
    return _msg_sensor_event.value.color_value;
  }

private:
  wippersnapper_sensor_SensorEvent _msg_sensor_event;
};
#endif // WS_SENSOR_MODEL_H