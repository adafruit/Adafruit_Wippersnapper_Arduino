/*!
 * @file model.cpp
 *
 * Model for the digitalio.proto message.
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
    @brief  DigitalIOModel constructor
*/
/***********************************************************************/
DigitalIOModel::DigitalIOModel() 
{
  memset(&_msg_dio_add, 0, sizeof(_msg_dio_add));
  memset(&_msg_dio_remove, 0, sizeof(_msg_dio_remove));
  memset(&_msg_dio_event, 0, sizeof(_msg_dio_event));
  memset(&_msg_dio_write, 0, sizeof(_msg_dio_write));
  // no-op
}

/***********************************************************************/
/*!
    @brief  DigitalIOModel destructor
*/
/***********************************************************************/
DigitalIOModel::~DigitalIOModel() {
  memset(&_msg_dio_add, 0, sizeof(_msg_dio_add));
  memset(&_msg_dio_remove, 0, sizeof(_msg_dio_remove));
  memset(&_msg_dio_event, 0, sizeof(_msg_dio_event));
  memset(&_msg_dio_write, 0, sizeof(_msg_dio_write));
}

/***********************************************************************/
/*!
    @brief  Parses a DigitalIOAdd message.
    @return DigitalIOAdd message object.
*/
/***********************************************************************/
wippersnapper_digitalio_DigitalIOAdd *DigitalIOModel::GetDigitalIOAddMsg() {
  return &_msg_dio_add;
}

/***********************************************************************/
/*!
    @brief  Parses a DigitalIORemove message.
    @param  stream
            The nanopb input stream.
    @return DigitalIORemove message object.
*/
/***********************************************************************/
bool DigitalIOModel::DecodeDigitalIORemove(pb_istream_t *stream) {
  // Zero-out the DigitalIORemove message struct. to ensure we don't have any
  // old data
  memset(&_msg_dio_remove, 0, sizeof(_msg_dio_remove));

  // Decode the stream into a DigitalIORemove message
  return pb_decode(stream, wippersnapper_digitalio_DigitalIORemove_fields,
                   &_msg_dio_remove);
}

/***********************************************************************/
/*!
    @brief  Gets a DigitalIOWrite message.
    @return DigitalIOWrite message object.
*/
/***********************************************************************/
wippersnapper_digitalio_DigitalIOWrite *DigitalIOModel::GetDigitalIOWriteMsg() {
  return &_msg_dio_write;
}

/***********************************************************************/
/*!
    @brief  Gets a DigitalIOEvent message.
    @return DigitalIOEvent message object.
*/
/***********************************************************************/
wippersnapper_digitalio_DigitalIOEvent *DigitalIOModel::GetDigitalIOEventMsg() {
  return &_msg_dio_event;
}

/***********************************************************************/
/*!
    @brief  Decodes a DigitalIOAdd message into the _msg_dio_add object
            from a nanopb stream.
    @param  stream
            The nanopb input stream.
    @return True if the DigitalIOAdd message was successfully decoded.
*/
/***********************************************************************/
bool DigitalIOModel::DecodeDigitalIOAdd(pb_istream_t *stream) {
  // Zero-out the DigitalIOAdd message struct. to ensure we don't have any old
  // data
  memset(&_msg_dio_add, 0, sizeof(_msg_dio_add));

  // Decode the stream into a DigitalIOAdd message
  return pb_decode(stream, wippersnapper_digitalio_DigitalIOAdd_fields,
                   &_msg_dio_add);
}

/***********************************************************************/
/*!
    @brief  Decodes a DigitalIOWrite message into the _msg_dio_write
            object from a nanopb stream.
    @param  stream
            The nanopb input stream.
    @return True if the DigitalIOWrite message was successfully decoded.
*/
/***********************************************************************/
bool DigitalIOModel::DecodeDigitalIOWrite(pb_istream_t *stream) {
  // Zero-out the DigitalIOWrite message struct. to ensure we don't have any old
  // data
  memset(&_msg_dio_write, 0, sizeof(_msg_dio_write));
  // Decode the stream into a DigitalIOWrite message
  return pb_decode(stream, wippersnapper_digitalio_DigitalIOWrite_fields,
                   &_msg_dio_write);
}

/***********************************************************************/
/*!
    @brief  Encodes a DigitalIOEvent message into the
            _msg_dio_event object.
    @param  pin_name
            The pin's name.
    @param  value
            The pin's value.
    @return True if the DigitalIOEvent message was successfully encoded.
            False if encoding resulted in a failure.
*/
/***********************************************************************/
bool DigitalIOModel::EncodeDigitalIOEvent(char *pin_name, bool value) {
  // Initialize the DigitalIOEvent
  memset(&_msg_dio_event, 0, sizeof(_msg_dio_event));
  // Fill the DigitalIOEvent
  strncpy(_msg_dio_event.pin_name, pin_name, sizeof(_msg_dio_event.pin_name));
  _msg_dio_event.has_value = true;
  // Fill the DigitalIOEvent's SensorEvent sub-message
  _msg_dio_event.value.type =
      wippersnapper_sensor_SensorType_SENSOR_TYPE_BOOLEAN;
  _msg_dio_event.value.which_value =
      wippersnapper_sensor_SensorEvent_bool_value_tag;
  _msg_dio_event.value.value.bool_value = value;

  // Encode the DigitalIOEvent message
  size_t sz_dio_event_msg;
  if (!pb_get_encoded_size(&sz_dio_event_msg,
                           wippersnapper_digitalio_DigitalIOEvent_fields,
                           &_msg_dio_event))
    return false;

  // Create an output stream
  uint8_t buf[sz_dio_event_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  // Encode the message
  return pb_encode(&msg_stream, wippersnapper_digitalio_DigitalIOEvent_fields,
                   &_msg_dio_event);
}