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
  _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_default;
}

/***********************************************************************/
/*!
    @brief  SensorModel destructor
*/
/***********************************************************************/
SensorModel::~SensorModel() {}

/***********************************************************************/
/*!
    @brief  Decodes a SensorEvent message into a SensorModel object.
    @param  stream
            The input stream
    @returns True if the SensorEvent message was successfully decoded,
            False otherwise.
*/
/***********************************************************************/
bool SensorModel::decodeSensorEvent(pb_istream_t *stream) {
  // Decode the stream into theSensorEvent message
  if (!pb_decode(stream, wippersnapper_sensor_SensorEvent_fields,
                 &_msg_sensor_event)) {
    return false;
  }
  return true;
}

/***********************************************************************/
/*!
    @brief  Clears the SensorEvent message.
*/
/***********************************************************************/
void SensorModel::clearSensorEvent() {
  _msg_sensor_event = wippersnapper_sensor_SensorEvent_init_default;
}

/***********************************************************************/
/*!
    @brief  Returns a SensorEvent message.
    @returns The SensorEvent message.
*/
/***********************************************************************/
wippersnapper_sensor_SensorEvent *SensorModel::getSensorEvent() {
  return &_msg_sensor_event;
}

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's type.
    @returns The type of the SensorEvent message, as a SensorType.
*/
/***********************************************************************/
wippersnapper_sensor_SensorType SensorModel::getSensorType() {
  return _msg_sensor_event.type;
}

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's which_value field.
    @returns The which_value field of the SensorEvent message.
*/
/***********************************************************************/
pb_size_t SensorModel::getSensorEventWhichValue() {
  return _msg_sensor_event.which_value;
}

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's float value field.
    @returns The float value of the SensorEvent message.
*/
/***********************************************************************/
float SensorModel::getValueFloat() {
  return _msg_sensor_event.value.float_value;
}

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's bool value field.
    @returns The bool value of the SensorEvent message.
*/
/***********************************************************************/
bool SensorModel::getValueBool() { return _msg_sensor_event.value.bool_value; }

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's vector value field.
    @returns The vector value of the SensorEvent message.
*/
/***********************************************************************/
wippersnapper_sensor_SensorEvent_SensorEvent3DVector
SensorModel::getValueVector() {
  return _msg_sensor_event.value.vector_value;
}

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's orientation value field.
    @returns The orientation value of the SensorEvent message.
*/
/***********************************************************************/
wippersnapper_sensor_SensorEvent_SensorEventOrientation
SensorModel::getValueOrientation() {
  return _msg_sensor_event.value.orientation_value;
}

/***********************************************************************/
/*!
    @brief  Returns the the SensorEvent message's color value field.
    @returns The color value of the SensorEvent message.
*/
/***********************************************************************/
wippersnapper_sensor_SensorEvent_SensorEventColor SensorModel::getValueColor() {
  return _msg_sensor_event.value.color_value;
}