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
  _driver_type = GPS_DRV_NONE;
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
 * @brief Detects if the GPS module is a MediaTek GPS module by querying its
 * firmware version.
 * @returns True if a MediaTek GPS module was detected, False otherwise.
 */
bool GPSController::DetectMediatek() {
  size_t buf_len = 83;  // NMEA sentence length + padding
  char buffer[buf_len]; // Holds the response from the GPS module, ideally a
                        // NMEA sentence

  // Query MediaTek firmware version
  _uart_hardware->GetHardwareSerial()->flush();
  _uart_hardware->GetHardwareSerial->println(CMD_MTK_QUERY_FW);
  // Wait for response
  uint16_t timeout = 1000; // 1 second timeout
  while (_uart_hardware->GetHardwareSerial()->available() < 100 && timeout--) {
    delay(1);
  }

  if (timeout == 0) {
    // TODO: Remove in production
    WS_DEBUG_PRINTLN("[gps] ERROR: Timeout from MediaTek GPS module!");
    return false;
  }

  // We found a response, let's verify that it's the expected PMTK_DK_RELEASE
  // command Read out the NMEA sentence string into a buffer
  for (size_t i = 0; i < buf_len; i++) {
    buffer[i] = _uart_hardware->GetHardwareSerial()->read();
  }

  // Compare the first 7 characters to the expected PMTK705 string
  if (strncmp(buffer, CMD_MTK_QUERY_FW_RESP, 7) != 0) {
    return false;
  }

  // Load Adafruit_GPS instance
  _ada_gps = Adafruit_GPS(_uart_hardware->GetHardwareSerial());
  if (!_ada_gps.begin(_uart_hardware->GetBaudRate())) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize Mediatek!");
    return false;
  }
  _driver_type = GPS_DRV_MTK;
  WS_DEBUG_PRINTLN("[gps] Detected MediaTek GPS module!");
  return true;
}

/*!
 * @brief Queries the GPS driver type by attempting to detect MediaTek or u-blox
 * GPS modules.
 * @returns True if the driver type was detected successfully, False otherwise.
 */
bool GPSController::QueryDriverType() {
  WS_DEBUG_PRINTLN("[gps] Attempting to detect GPS driver type...");
  if (!DetectMediatek()) {
    WS_DEBUG_PRINTLN("[gps] Failed to detect mediatek module, attempting to "
                     "detect u-blox GPS module...");
    // If we didn't detect MediaTek, let's try to detect u-blox
    // TODO: Implement u-blox detection logic here
  }
}

bool GPSController::begin() {
  if (_iface_type == GPS_IFACE_NONE) {
    WS_DEBUG_PRINTLN("[gps] ERROR: No interface set for GPS!");
    return false;
  }

  if (_iface_type == GPS_IFACE_UART_HW) {
    // Attempt to detect the hw type from a predefined command set sent over
    // UART
    // TODO: Maybe this should be abstracted to a GPS Hardware class but
    // whatever, for now we'll try this as a MVP approach
    WS_DEBUG_PRINTLN("[gps] Initializing GPS with Hardware Serial...");

    // return _ada_gps.begin(_uart_hardware->GetBaudRate());
  } else {
    WS_DEBUG_PRINTLN("[gps] ERROR: Unimplemented interface type for begin!");
    return false;
  }
}

/*!
 * @brief Handles GPS configuration messages.
 * @param stream A pointer to the pb_istream_t stream.
 * @returns True if the GPS configuration was handled successfully, False
 * otherwise.
 */
bool GPSController::Handle_GPSConfig(pb_istream_t *stream) {
  WS_DEBUG_PRINTLN("[gps] Decoding GPSConfig message...");
  if (!_gps_model->DecodeGPSConfig(stream)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to decode GPSConfig message!");
    return false;
  }

  // NOTE: GPSConfig just stores the commands from IO to send to the GPS device,
  // it does not store anything else!
  // TODO: This is debug-only, remove in production!
  for (pb_size_t i = 0; i < _gps_model->GetGPSConfigMsg()->commands_count;
       i++) {
    WS_DEBUG_PRINT("[gps] Processing Command: ");
    WS_DEBUG_PRINTLN(_gps_model->GetGPSConfigMsg()->commands[i]);
    Send_Command(_gps_model->GetGPSConfigMsg()->commands[i]);
  }
  // TODO: Implement the logic to handle the GPS configuration
  return false;
}

/*!
 * @brief Sends a command to the GPS module.
 * @param command The command to send to the GPS module.
 * @param timeout_ms The timeout in milliseconds for the command to complete.
 *                   Default is 1000 milliseconds.
 * @returns True if the command was sent successfully, False otherwise.
 */
/* bool GPSController::Send_Command(const char *command,
                                 unsigned long timeout_ms) {
  if (!IsAvailable()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: GPS interface is not available!");
    return false;
  }

  // Check which type of interface we have
  if (_iface_type == GPS_IFACE_UART_HW) {
    WS_DEBUG_PRINT("[gps] Sending command via Hardware Serial: ");
    WS_DEBUG_PRINTLN(command);
    long start_time = millis();
    // Wait for the hardware serial to be ready
    // TODO: Send command via HardwareSerial
    _uart_hardware->GetHardwareSerial()->println(command);
    while ()
  } else {
    WS_DEBUG_PRINT("[gps] Unknown interface type, cannot send command: ");
    WS_DEBUG_PRINTLN(command);
    return false;
  }
} */

/*!
 * @brief Removes a GPS device by its ID.
 * @param id The ID of the GPS device to remove.
 * @returns True if the device was removed successfully, False otherwise.
 */
bool RemoveGPSDevice(const char *id) { return false; }

void update() {
  // TODO: Implement!
}