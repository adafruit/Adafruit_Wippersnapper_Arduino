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
 * @brief Sets a UART hardware interface for the GPS controller.
 * @param uart_hardware Pointer to the UARTHardware instance.
 * @returns True if the interface was set successfully, False otherwise.
 */
bool GPSController::SetInterface(UARTHardware *uart_hardware) {
  if (uart_hardware == nullptr) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Provided UART instance is undefined!");
    return false;
  }
  _uart_hardware = uart_hardware;
}

/*!
 * @brief Handles GPS configuration messages.
 * @param stream A pointer to the pb_istream_t stream.
 * @returns True if the GPS configuration was handled successfully, False otherwise.
 */
bool GPSController::Handle_GPSConfig(pb_istream_t *stream) {
  WS_DEBUG_PRINTLN("[gps] Decoding GPSConfig message...");
  if (!_gps_model->DecodeGPSConfig(stream)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to decode GPSConfig message!");
    return false;
  }
  // NOTE: GPSConfig just stores the commands from IO to send to the GPS device, it does
  // not store anything else!
  for (pb_size_t i = 0; i < _gps_model->GetGPSConfigMsg()->commands_count; i++) {
    WS_DEBUG_PRINT("[gps] Command: ");
    WS_DEBUG_PRINTLN(_gps_model->GetGPSConfigMsg()->commands[i]);
  }
  // TODO: Implement the logic to handle the GPS configuration
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