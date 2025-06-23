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

  // Clean up GPS instance
  if (_ada_gps) {
    delete _ada_gps;
    _ada_gps = nullptr;
  }
}

/*!
 * @brief Sets a UART hardware interface for the GPS controller.
 * @param serial
 *        Pointer to a HardwareSerial instance.
 * @returns True if the interface was set successfully, False otherwise.
 */
bool GPSController::SetInterface(HardwareSerial *serial) {
  if (serial == nullptr)
    return false;
  // Set the hardware serial interface
  _hw_serial = serial;
  _iface_type = GPS_IFACE_UART_HW;
  return true;
}

/*!
 * @brief Attempts to initialize the GPS device based on the configured
 * interface type.
 * @returns True if the GPS device was initialized successfully, False
 * otherwise.
 */
bool GPSController::begin() {
  // Validate if the interface type is set
  if (_iface_type == GPS_IFACE_NONE) {
    WS_DEBUG_PRINTLN("[gps] ERROR: No interface type configured!");
    return false;
  }

  // Attempt to set the GPS interface type based on the hardware
  if (!QueryModuleType()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to query GPS module type!");
    return false;
  }

  WS_DEBUG_PRINTLN(
      "[gps] GPS module type detected successfully and ready for commands!");
  return true;
}

/*!
 * @brief Queries the GPS driver type by attempting to detect MediaTek or u-blox
 * GPS modules.
 * @returns True if the driver type was detected successfully, False otherwise.
 */
bool GPSController::QueryModuleType() {
  // Validate if the interface is set
  if (_iface_type == GPS_IFACE_NONE) {
    WS_DEBUG_PRINTLN("[gps] ERROR: No interface configured for GPS!");
    return false;
  }
  WS_DEBUG_PRINTLN("[gps] Attempting to detect GPS module type...");

  // Try to detect MediaTek GPS module
  if (DetectMediatek()) {
    WS_DEBUG_PRINTLN("[gps] Using MediaTek GPS driver!");
    return true;
  }

  WS_DEBUG_PRINTLN("[gps] Failed to detect MTK GPS, attempting to detect "
                   "u-blox GPS module...");
  // TODO: Implement u-blox detection here
  // if (DetectUblox()) {
  //   return true;
  // }

  WS_DEBUG_PRINTLN("[gps] ERROR: Failed to detect GPS driver type, attempting "
                   "to use generic driver!");
  // TODO: Implement generic NMEA GPS driver detection

  // No responses from the GPS module over the defined iface, so we bail out
  WS_DEBUG_PRINTLN("[gps] ERROR: No GPS driver type detected!");
  return false;
}

/*!
 * @brief Detects if the GPS module is a MediaTek GPS module by querying its
 * firmware version.
 * @returns True if a MediaTek GPS module was detected, False otherwise.
 */
bool GPSController::DetectMediatek() {
  if (_iface_type != GPS_IFACE_UART_HW) {
    WS_DEBUG_PRINTLN("[gps] ERROR: MediaTek GPS module only supports Hardware "
                     "Serial interface!");
    return false;
  }

  // Query MediaTek firmware version
  _hw_serial->flush();
  _hw_serial->println(CMD_MTK_QUERY_FW);
  // Wait for response
  uint16_t timeout = 1000; // 1 second timeout
  while (_hw_serial->available() < MAX_NEMA_SENTENCE_LEN && timeout--) {
    delay(1);
  }

  if (timeout == 0) {
    // TODO: Remove in production
    WS_DEBUG_PRINTLN("[gps] ERROR: Timeout from MediaTek GPS module!");
    return false;
  }

  // We found a response, let's verify that it's the expected PMTK_DK_RELEASE
  // command by reading out the NMEA sentence string into a buffer
  size_t buf_len = MAX_NEMA_SENTENCE_LEN + 3; // +3 for \r\n and null terminator
  char buffer[buf_len];
  size_t available = _hw_serial->available();
  size_t bytes_to_read = min(available, buf_len - 1);
  // Print the two out
  WS_DEBUG_PRINT("[gps] Reading MediaTek GPS response: ");
  WS_DEBUG_PRINT(available);
  WS_DEBUG_PRINT(" bytes, reading ");
  WS_DEBUG_PRINTLN(bytes_to_read);
  for (size_t i = 0; i < bytes_to_read; i++) {
    buffer[i] = _hw_serial->read();
  }
  buffer[bytes_to_read] = '\0';
  WS_DEBUG_PRINT("[gps] MediaTek GPS response: ");
  WS_DEBUG_PRINTLN(buffer);
  // did we get the expected PMTK705 string?
  if (strncmp(buffer, CMD_MTK_QUERY_FW_RESP, 8) != 0) {
    return false;
  }

  // Attempt to use Adafruit_GPS
  if (_ada_gps != nullptr) {
    delete _ada_gps; // Clean up previous instance if it exists
  }
  _ada_gps = new Adafruit_GPS(_hw_serial);
  if (!_ada_gps->begin(_hw_serial->baudRate())) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize Mediatek!");
    return false;
  }
  _driver_type = GPS_DRV_MTK;
  return true;
}
