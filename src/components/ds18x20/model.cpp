/*!
 * @file model.cpp
 *
 * Model for the ds18x20.proto message.
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
    @brief  DS18X20Model constructor
*/
/***********************************************************************/
DS18X20Model::DS18X20Model() {
  // Initialize the DS18X20 messages
  _msg_DS18x20Add = wippersnapper_ds18x20_Ds18x20Add_init_zero;
  _msg_DS18x20Added = wippersnapper_ds18x20_Ds18x20Added_init_zero;
  _msg_DS18x20Remove = wippersnapper_ds18x20_Ds18x20Remove_init_zero;
}

/***********************************************************************/
/*!
    @brief  DS18X20Model destructor
*/
/***********************************************************************/
DS18X20Model::~DS18X20Model() {}

/***********************************************************************/
/*!
    @brief  Attempts to decode a Ds18x20Add message from the broker.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully decoded, False otherwise.
*/
/***********************************************************************/
bool DS18X20Model::DecodeDS18x20Add(pb_istream_t *stream) {
  _msg_DS18x20Add = wippersnapper_ds18x20_Ds18x20Add_init_zero;
  // Attempt to decode the stream into a Ds18x20Add message
  return pb_decode(stream, wippersnapper_ds18x20_Ds18x20Add_fields,
                   &_msg_DS18x20Add);
}

/***********************************************************************/
/*!
    @brief  Gets a pointer to the Ds18x20Add message.
    @return Pointer to the Ds18x20Add message.
*/
/***********************************************************************/
wippersnapper_ds18x20_Ds18x20Add *DS18X20Model::GetDS18x20AddMsg() {
  return &_msg_DS18x20Add;
}

/***********************************************************************/
/*!
    @brief  Returns a pointer to the Ds18x20Added message.
    @return Pointer to the Ds18x20Added message.
*/
/***********************************************************************/
wippersnapper_ds18x20_Ds18x20Added *DS18X20Model::GetDS18x20AddedMsg() {
  return &_msg_DS18x20Added;
}

/***********************************************************************/
/*!
    @brief  Encodes a Ds18x20Added message.
    @param  onewire_pin
            The OneWire bus pin to add.
    @param  is_init
            True if the sensor was successfully initialized,
            False otherwise.
    @return True if the message was successfully encoded,
            False otherwise.
*/
/***********************************************************************/
bool DS18X20Model::EncodeDS18x20Added(char *onewire_pin, bool is_init) {
  // Fill the Ds18x20Added message
  _msg_DS18x20Added = wippersnapper_ds18x20_Ds18x20Added_init_zero;
  _msg_DS18x20Added.is_initialized = is_init;
  strcpy(_msg_DS18x20Added.onewire_pin, onewire_pin);

  // Encode the Ds18x20Added message
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, wippersnapper_ds18x20_Ds18x20Added_fields,
                           &_msg_DS18x20Added))
    return false;

  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_ds18x20_Ds18x20Added_fields,
                   &_msg_DS18x20Added);
}

/*************************************************************************/
/*!
    @brief  Attempts to decode a Ds18x20Remove message from the broker.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully decoded, False otherwise.
*/
/*************************************************************************/
bool DS18X20Model::DecodeDS18x20Remove(pb_istream_t *stream) {
  _msg_DS18x20Remove = wippersnapper_ds18x20_Ds18x20Remove_init_zero;
  return pb_decode(stream, wippersnapper_ds18x20_Ds18x20Remove_fields,
                   &_msg_DS18x20Remove);
}

/*************************************************************************/
/*!
    @brief  Gets a pointer to the Ds18x20Remove message.
    @return Pointer to the Ds18x20Remove message.
*/
/*************************************************************************/
wippersnapper_ds18x20_Ds18x20Remove *DS18X20Model::GetDS18x20RemoveMsg() {
  return &_msg_DS18x20Remove;
}

/*************************************************************************/
/*!
    @brief  Gets a pointer to the Ds18x20Event message.
    @return Pointer to the Ds18x20Event message.
*/
/*************************************************************************/
wippersnapper_ds18x20_Ds18x20Event *DS18X20Model::GetDS18x20EventMsg() {
  return &_msg_DS18x20Event;
}

/*************************************************************************/
/*!
    @brief  Encodes a Ds18x20Event message.
    @param  onewire_pin
            The OneWire bus' pin where the sensor is located.
    @param  sensor_events_count
            The number of sensorevent messages to encode.
    @param  sensor_events
            The sensorevent messages to encode.
    @return True if the message was successfully encoded, False otherwise.
*/
/*************************************************************************/
bool DS18X20Model::EncodeDs18x20Event(
    char *onewire_pin, pb_size_t sensor_events_count,
    const wippersnapper_sensor_SensorEvent sensor_events[2]) {
  // Fill the Ds18x20Event message
  _msg_DS18x20Event = wippersnapper_ds18x20_Ds18x20Event_init_zero;
  strcpy(_msg_DS18x20Event.onewire_pin, onewire_pin);
  _msg_DS18x20Event.sensor_events_count = sensor_events_count;
  // Fill with sensor_events
  for (int i = 0; i < _msg_DS18x20Event.sensor_events_count; i++) {
    _msg_DS18x20Event.sensor_events[i] = sensor_events[i];
  }
  // Encode and return the Ds18x20Event message
  size_t sz_msg;
  if (!pb_get_encoded_size(&sz_msg, wippersnapper_ds18x20_Ds18x20Event_fields,
                           &_msg_DS18x20Event))
    return false;
  uint8_t buf[sz_msg];
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  return pb_encode(&msg_stream, wippersnapper_ds18x20_Ds18x20Event_fields,
                   &_msg_DS18x20Event);
}

/*************************************************************************/
/*!
    @brief  Initializes the Ds18x20Event message.
*/
/*************************************************************************/
void DS18X20Model::InitDS18x20EventMsg() {
  _msg_DS18x20Event = wippersnapper_ds18x20_Ds18x20Event_init_zero;
  _msg_DS18x20Event.sensor_events_count = 0;
}

/*************************************************************************/
/*!
    @brief  Adds a "sensor event" to the Ds18x20Event message.
    @param  sensor_type
            The event's SensorType.
    @param  sensor_value
            The event's value.
*/
/*************************************************************************/
void DS18X20Model::addSensorEvent(wippersnapper_sensor_SensorType sensor_type,
                                  float sensor_value) {
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count].type =
      sensor_type;
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count]
      .which_value = wippersnapper_sensor_SensorEvent_float_value_tag;
  _msg_DS18x20Event.sensor_events[_msg_DS18x20Event.sensor_events_count]
      .value.float_value = sensor_value;
  _msg_DS18x20Event.sensor_events_count++;
}