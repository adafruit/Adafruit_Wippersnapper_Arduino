/*!
 * @file src/components/gps/model.cpp
 *
 * Model implementation for the GPS.proto message.
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
#include "model.h"

/*!
 * @brief Constructor for a GPSModel object.
 */
GPSModel::GPSModel() {
  memset(&_msg_gps_config, 0, sizeof(_msg_gps_config));
  memset(&_msg_gpgga_response, 0, sizeof(_msg_gpgga_response));
  memset(&_msg_gpsrmc_response, 0, sizeof(_msg_gpsrmc_response));
}

/*!
 * @brief Destructor for a GPSModel object.
 */
GPSModel::~GPSModel() {
  memset(&_msg_gps_config, 0, sizeof(_msg_gps_config));
  memset(&_msg_gpgga_response, 0, sizeof(_msg_gpgga_response));
  memset(&_msg_gpsrmc_response, 0, sizeof(_msg_gpsrmc_response));
}

/*!
 * @brief Decodes a GPSConfig message from an input stream.
 * @param stream A pointer to the pb_istream_t stream.
 * @returns True if the GPSConfig message was decoded successfully, False otherwise.
 */
bool DecodeGPSConfig(pb_istream_t *stream) {
  return pb_decode(stream, wippersnapper_gps_GPSConfig_fields, &_msg_gps_config);
}

/*!
 * @brief Returns a pointer to the GPSConfig message.
 * @returns Pointer to the GPSConfig message.
 */
wippersnapper_gps_GPSConfig *GetGPSConfigMsg() {
  return &_msg_gps_config;
}

/*!
 * @brief Encodes a GPGGA response message.
 * @returns True if the GPGGA response message was encoded successfully, False otherwise.
 */
bool EncodeGPGGAResponse() {
  // TODO: Implement the encoding logic for GPGGAResponse
  return false;
}

/*!
 * @brief Returns a pointer to the GPGGA response message.
 * @returns Pointer to the GPGGA response message.
 */
wippersnapper_gps_GPGGAResponse *GetGPGGAResponseMsg() {
  return &_msg_gpgga_response;
}

/*!
 * @brief Gets the GPGGA response message.
 */
wippersnapper_gps_GPSRMCResponse *GetGPSRMCResponseMsg() {
  return &_msg_gpsrmc_response;
}
