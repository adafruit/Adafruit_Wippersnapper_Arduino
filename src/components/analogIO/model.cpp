/*!
 * @file model.cpp
 *
 * Interfaces for the analogio.proto API
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
    @brief  AnalogIOModel constructor
*/
/***********************************************************************/
AnalogIOModel::AnalogIOModel() {
  _msg_AnalogioAdd = wippersnapper_analogio_AnalogIOAdd_init_default;
}

/***********************************************************************/
/*!
    @brief  AnalogIOModel destructor
*/
/***********************************************************************/
AnalogIOModel::~AnalogIOModel() {}

/***********************************************************************/
/*!
    @brief  Decodes an AnalogIOAdd message from a stream into an
            AnalogIOAdd message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
/***********************************************************************/
bool AnalogIOModel::DecodeAnalogIOAdd(pb_istream_t *stream) {
  // Zero-out the AnalogIOAdd message struct. to ensure we don't have any old
  // data
  _msg_AnalogioAdd = wippersnapper_analogio_AnalogIOAdd_init_default;
  // Decode the stream into a AnalogIOAdd message
  return pb_decode(stream, wippersnapper_analogio_AnalogIOAdd_fields,
                   &_msg_AnalogioAdd);
}

/***********************************************************************/
/*!
    @brief  Gets an AnalogIOAdd message struct.
    @return Pointer to an AnalogIOAdd message struct.
*/
/***********************************************************************/
wippersnapper_analogio_AnalogIOAdd *AnalogIOModel::GetAnalogIOAddMsg() {
  return &_msg_AnalogioAdd;
}

/***********************************************************************/
/*!
    @brief  Decodes an AnalogIORemove message from a stream into an
            AnalogIORemove message struct.
    @param stream
           The pb_istream_t stream to decode.
    @return True if successful, False otherwise.
*/
/***********************************************************************/
bool AnalogIOModel::DecodeAnalogIORemove(pb_istream_t *stream) {
  // Zero-out the AnalogIORemove message struct. to ensure we don't have any old
  // data
  _msg_AnalogioRemove = wippersnapper_analogio_AnalogIORemove_init_default;
  // Decode the stream into a AnalogIORemove message
  return pb_decode(stream, wippersnapper_analogio_AnalogIORemove_fields,
                   &_msg_AnalogioRemove);
}

/*****************************************************************************/
/*!
    @brief  Gets an AnalogIORemove message struct.
    @return Pointer to an AnalogIORemove message struct.
*/
/*****************************************************************************/
wippersnapper_analogio_AnalogIORemove *AnalogIOModel::GetAnalogIORemoveMsg() {
  return &_msg_AnalogioRemove;
}

/*****************************************************************************/
/*!
    @brief  Gets an AnalogIOEvent message struct.
    @return Pointer to an AnalogIOEvent message struct.
*/
/*****************************************************************************/
wippersnapper_analogio_AnalogIOEvent *AnalogIOModel::GetAnalogIOEvent() {
  return &_msg_AnalogioEvent;
}

/*****************************************************************************/
/*!
    @brief  Encodes an AnalogIOEvent message.
    @param pin_name
           The requested pin's name.
    @param pin_value
           The value of the pin.
    @param read_type
           The type of sensor event to encode.
    @return True if successful, False otherwise.
*/
/*****************************************************************************/
bool AnalogIOModel::EncodeAnalogIOEvent(
    char *pin_name, float pin_value,
    wippersnapper_sensor_SensorType read_type) {
  // Initialize the AnalogIOEvent message to default values
  _msg_AnalogioEvent = wippersnapper_analogio_AnalogIOEvent_init_zero;
  // Fill the AnalogIOEvent message's fields
  strncpy(_msg_AnalogioEvent.pin_name, pin_name,
          sizeof(_msg_AnalogioEvent.pin_name));
  _msg_AnalogioEvent.has_sensor_event = true;
  _msg_AnalogioEvent.sensor_event.type = read_type;
  _msg_AnalogioEvent.sensor_event.which_value =
      wippersnapper_sensor_SensorEvent_float_value_tag;
  _msg_AnalogioEvent.sensor_event.value.float_value = pin_value;

  // Obtain size of an encoded AnalogIOEvent message
  size_t sz_aio_event_msg;
  if (!pb_get_encoded_size(&sz_aio_event_msg,
                           wippersnapper_analogio_AnalogIOEvent_fields,
                           &_msg_AnalogioEvent))
    return false;

  // Encode the AnalogIOEvent message
  uint8_t buf[sz_aio_event_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_analogio_AnalogIOEvent_fields,
                   &_msg_AnalogioEvent);
}

/********************************************************************************/
/*!
    @brief  Encodes an AnalogIOEvent message with a raw pin value.
    @param pin_name
           The requested pin's name.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
/********************************************************************************/
bool AnalogIOModel::EncodeAnalogIOEventRaw(char *pin_name, float pin_value) {
  return EncodeAnalogIOEvent(pin_name, pin_value,
                             wippersnapper_sensor_SensorType_SENSOR_TYPE_RAW);
}

/********************************************************************************/
/*!
    @brief  Encodes an AnalogIOEvent message with a voltage pin value.
    @param pin_name
           The requested pin's name.
    @param pin_value
           The value of the pin.
    @return True if successful, False otherwise.
*/
/********************************************************************************/
bool AnalogIOModel::EncodeAnalogIOEventVoltage(char *pin_name,
                                               float pin_value) {
  return EncodeAnalogIOEvent(
      pin_name, pin_value, wippersnapper_sensor_SensorType_SENSOR_TYPE_VOLTAGE);
}