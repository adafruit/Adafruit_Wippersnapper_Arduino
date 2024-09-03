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
DigitalIOModel::DigitalIOModel() {
  _msg_dio_add = wippersnapper_digitalio_DigitalIOAdd_init_default;
  _msg_dio_remove = wippersnapper_digitalio_DigitalIORemove_init_default;
  _msg_dio_event = wippersnapper_digitalio_DigitalIOEvent_init_default;
  _msg_dio_write = wippersnapper_digitalio_DigitalIOWrite_init_default;
}

/***********************************************************************/
/*!
    @brief  DigitalIOModel destructor
*/
/***********************************************************************/
DigitalIOModel::~DigitalIOModel() {}

bool DigitalIOModel::DecodeDigitalIOAdd(pb_istream_t *stream) {
  return pb_decode(stream, wippersnapper_digitalio_DigitalIOAdd_fields,
                   &_msg_dio_add);
}

void DigitalIOModel::ParseDigitalIOAdd() {
  // TODO
}