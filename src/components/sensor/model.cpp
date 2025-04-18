/*!
 * @file src/components/sensor/model.cpp
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
  memset(&_msg_sensor_event, 0, sizeof(_msg_sensor_event));
  // no-op
}

/***********************************************************************/
/*!
    @brief  SensorModel destructor
*/
/***********************************************************************/
SensorModel::~SensorModel() {
  // Zero-out the SensorEvent message
  memset(&_msg_sensor_event, 0, sizeof(_msg_sensor_event));
}