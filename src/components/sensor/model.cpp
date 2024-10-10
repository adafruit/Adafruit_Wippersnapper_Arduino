/*!
 * @file model.cpp
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
#include "model.h"

/***********************************************************************/
/*!
    @brief  SensorModel constructor
*/
/***********************************************************************/
SensorModel::SensorModel() {
  _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_zero;
}

/***********************************************************************/
/*!
    @brief  SensorModel destructor
*/
/***********************************************************************/
SensorModel::~SensorModel() {
  // Zero-out the SensorEvent message
  _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_zero;
}

wippersnapper_sensor_SensorEvent *SensorModel::GetSensorEventMessage() {
    return &_msg_sensor_event;
}

bool SensorModel::EncodeSensorEventMessage() {
    // Obtain size of the class' SensorEvent message
    size_t sz_msg;
    if (!pb_get_encoded_size(&sz_msg, wippersnapper_sensor_SensorEvent_fields, &_msg_sensor_event))
      return false;

    // Encode the class' SensorEvent message
    uint8_t buf[sz_msg];
    pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
    return pb_encode(&msg_stream, wippersnapper_sensor_SensorEvent_fields,
                    &_msg_sensor_event);
}


void SensorModel::CreateSensorEventFloat(wippersnapper_sensor_SensorType sensor_type, float sensor_value) {
    // Zero-out the previous SensorEvent message
    _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_zero;
    // Fill in the SensorEvent message
    _msg_sensor_event.type = sensor_type;
    _msg_sensor_event.which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
    _msg_sensor_event.value.float_value = sensor_value;
}