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
  _iface_type = GPS_IFACE_NONE;
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
  // Determine iface type
  if (_uart_hardware->isHardwareSerial()) {
    _iface_type = GPS_IFACE_UART_HW;
  } else if (_uart_hardware->isSoftwareSerial()) {
    _iface_type = GPS_IFACE_UART_SW;
  } else {
    WS_DEBUG_PRINTLN("[gps] ERROR: Unsupported UART interface type!");
    return false;
  }
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
  // TODO: This is debug-only, remove in production!
  for (pb_size_t i = 0; i < _gps_model->GetGPSConfigMsg()->commands_count; i++) {
    WS_DEBUG_PRINT("[gps] Processing Command: ");
    WS_DEBUG_PRINTLN(_gps_model->GetGPSConfigMsg()->commands[i]);
    Send_Command(_gps_model->GetGPSConfigMsg()->commands[i]);
  }
  // TODO: Implement the logic to handle the GPS configuration
  return false;
}

/*!
 * @brief Checks if the GPS module's interface is available.
 * @returns True if the module's interface is available, False otherwise.
 */
bool GPSController::IsAvailable() {
  if (! _uart_hardware == nullptr)
    return true;
  WS_DEBUG_PRINTLN("[gps] ERROR: No UART hardware interface set!");
  return false;
}

/*!
 * @brief Sends a command to the GPS module.
 * @param command The command to send to the GPS module.
 * @param timeout_ms The timeout in milliseconds for the command to complete.
 *                   Default is 1000 milliseconds.
 * @returns True if the command was sent successfully, False otherwise.
 */
bool GPSController::Send_Command(const char *command, unsigned long timeout_ms) { 
  if (!IsAvailable()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: GPS interface is not available!");
    return false;
  }

  // Check which type of interface we have
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