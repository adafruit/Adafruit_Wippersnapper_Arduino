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
  memset(&_msg_gps_event, 0, sizeof(_msg_gps_event));
}

/*!
 * @brief Destructor for a GPSModel object.
 */
GPSModel::~GPSModel() {
  memset(&_msg_gps_config, 0, sizeof(_msg_gps_config));
  memset(&_msg_gps_event, 0, sizeof(_msg_gps_event));
}

/*!
 * @brief Decodes a GPSConfig message from an input stream.
 * @param stream A pointer to the pb_istream_t stream.
 * @returns True if the GPSConfig message was decoded successfully, False
 * otherwise.
 */
bool GPSModel::DecodeGPSConfig(pb_istream_t *stream) {
  return pb_decode(stream, wippersnapper_gps_GPSConfig_fields,
                   &_msg_gps_config);
}

/*!
 * @brief Returns a pointer to the GPSConfig message.
 * @returns Pointer to the GPSConfig message.
 */
wippersnapper_gps_GPSConfig *GPSModel::GetGPSConfigMsg() {
  return &_msg_gps_config;
}

/*!
 * @brief Creates a new GPSEvent message and initializes it.
 * @NOTE: This function will clear an existing GPSEvent message. Only call when
 * you are creating a NEW gps event, not modifying an existing one.
 */
void GPSModel::CreateGPSEvent() {
  // Zero-out whatever was previously in the GPSEvent message
  memset(&_msg_gps_event, 0, sizeof(_msg_gps_event));
  // Create new GPSEvent message with initializer
  _msg_gps_event = wippersnapper_gps_GPSEvent_init_zero;
  _msg_gps_event.gga_responses_count = 0;
  _msg_gps_event.rmc_responses_count = 0;
}

/*!
 * @brief Creates a GPSDateTime message with the provided parameters.
 * @param hour GMT hour of the day (0-23).
 * @param minute GMT minute of the hour (0-59).
 * @param seconds GMT seconds of the minute (0-59).
 * @param milliseconds GMT milliseconds (0-999).
 * @param day GMT day of the month (1-31).
 * @param month GMT month of the year (1-12).
 * @param year GMT year (e.g., 25).
 * @returns A wippersnapper_gps_GPSDateTime message.
 */
wippersnapper_gps_GPSDateTime
GPSModel::CreateGpsDatetime(uint8_t hour, uint8_t minute, uint8_t seconds,
                            uint8_t milliseconds, uint8_t day, uint8_t month,
                            uint8_t year) {
  wippersnapper_gps_GPSDateTime datetime;
  // Fill in the datetime structure with the provided values
  datetime.hour = (uint32_t)(hour);
  datetime.minute = (uint32_t)(minute);
  datetime.seconds = (uint32_t)(seconds);
  datetime.milliseconds = (uint32_t)(milliseconds);
  datetime.day = (uint32_t)(day);
  datetime.month = (uint32_t)(month);
  datetime.year = (uint32_t)(year);
  return datetime;
}

bool GPSModel::AddGpsEventRMC(wippersnapper_gps_GPSDateTime *datetime,
                              uint8_t fix_status, float lat, char *lat_dir,
                              float lon, char *lon_dir, float speed,
                              float angle) {
  // Check if we've reached the maximum number of RMC responses
  if (_msg_gps_event.rmc_responses_count >= MAX_COUNT_RMC_GGA) {
    return false;
  }

  // Validate pointers have been provided correctly
  if (!lat_dir || !lon_dir) {
    return false;
  }

  wippersnapper_gps_GPSRMCResponse rmc_response;
  rmc_response = wippersnapper_gps_GPSRMCResponse_init_zero;
  // Assign the datetime, if provided
  if (datetime) {
    rmc_response.has_datetime = true;
    rmc_response.datetime = *datetime;
  } else {
    rmc_response.has_datetime = false;
  }

  // Determine the fix status
  if (fix_status == 1 || fix_status == 2) {
    rmc_response.fix_status[0] = 'A'; // Active fix
  } else {
    rmc_response.fix_status[0] = 'V'; // Void fix
  }
  rmc_response.fix_status[1] = '\0';

  // Fill lat/lon and direction
  snprintf(rmc_response.lat, sizeof(rmc_response.lat), "%.6f", lat);
  snprintf(rmc_response.lon, sizeof(rmc_response.lon), "%.6f", lon);
  strncpy(rmc_response.lat_dir, lat_dir, sizeof(rmc_response.lat_dir) - 1);
  rmc_response.lat_dir[sizeof(rmc_response.lat_dir) - 1] = '\0';
  strncpy(rmc_response.lon_dir, lon_dir, sizeof(rmc_response.lon_dir) - 1);
  rmc_response.lon_dir[sizeof(rmc_response.lon_dir) - 1] = '\0';

  // Fill current speed over ground, in knots
  snprintf(rmc_response.speed, sizeof(rmc_response.speed), "%.1f", speed);
  // Fill course in degrees from true north
  snprintf(rmc_response.angle, sizeof(rmc_response.angle), "%.1f", angle);

  _msg_gps_event.rmc_responses[_msg_gps_event.rmc_responses_count] =
      rmc_response;
  _msg_gps_event.rmc_responses_count++;
  return true;
}

bool GPSModel::AddGpsEventGGA(wippersnapper_gps_GPSDateTime *datetime,
                              uint8_t fix_status, float lat, char *lat_dir,
                              float lon, char *lon_dir, uint8_t num_sats,
                              float hdop, float alt, float geoid_height) {

  // Check if we've reached the maximum number of RMC responses
  if (_msg_gps_event.gga_responses_count >= MAX_COUNT_RMC_GGA) {
    return false;
  }

  // Validate pointers have been provided correctly
  if (!lat_dir || !lon_dir) {
    return false;
  }

  wippersnapper_gps_GPGGAResponse gga_response;
  gga_response = wippersnapper_gps_GPGGAResponse_init_zero;
  // Assign the datetime, if provided
  if (datetime) {
    gga_response.has_datetime = true;
    gga_response.datetime = *datetime;
  } else {
    gga_response.has_datetime = false;
  }

  // Fill lat/lon and direction
  snprintf(gga_response.lat, sizeof(gga_response.lat), "%.6f", lat);
  snprintf(gga_response.lon, sizeof(gga_response.lon), "%.6f", lon);
  strncpy(gga_response.lat_dir, lat_dir, sizeof(gga_response.lat_dir) - 1);
  gga_response.lat_dir[sizeof(gga_response.lat_dir) - 1] = '\0';
  strncpy(gga_response.lon_dir, lon_dir, sizeof(gga_response.lon_dir) - 1);
  gga_response.lon_dir[sizeof(gga_response.lon_dir) - 1] = '\0';

  // Determine the fix quality
  gga_response.fix_quality = (uint32_t)fix_status;
  // Fill number of satellites in use
  gga_response.num_satellites = (uint32_t)num_sats;

  // Fill horizontal dilution of precision
  snprintf(gga_response.hdop, sizeof(gga_response.hdop), "%.1f", hdop);
  // Fill altitude in meters above MSL
  snprintf(gga_response.altitude, sizeof(gga_response.altitude), "%.1f", alt);
  // Fill geoid height in meters
  snprintf(gga_response.geoid_height, sizeof(gga_response.geoid_height), "%.1f",
           geoid_height);

  _msg_gps_event.gga_responses[_msg_gps_event.gga_responses_count] =
      gga_response;
  _msg_gps_event.gga_responses_count++;
  return true;
}