/*!
 * @file src/components/gps/controller.cpp
 *
 * Controller for WipperSnapper's GPS component, bridges between the GPS.proto
 * API, the model, and the hardware layer.
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
#include "controller.h"

/*!
 * @brief Constructor
 */
GPSController::GPSController() {
  _gps_model = new GPSModel();
}

/*!
 * @brief Destructor
 */
GPSController::~GPSController() {
  // Clean up model
  if (_gps_model) {
    delete _gps_model;
    _gps_model = nullptr;
  }
}

/*!
 * @brief Handles GPS configuration messages.
 * @param stream A pointer to the pb_istream_t stream.
 * @returns True if the GPS configuration was handled successfully, False otherwise.
 */
bool GPSController::Handle_GPSConfig(pb_istream_t *stream) {
  // TODO: Implement the logic to handle GPSAdd
  return false;
}

/*!
 * @brief Removes a GPS device by its ID.
 * @param id The ID of the GPS device to remove.
 * @returns True if the device was removed successfully, False otherwise.
 */
bool RemoveGPSDevice(const char *id) {
  return false;
}


void update() {
  // TODO: Implement! 
}