/*!
 * @file src/components/gps/controller.cpp
 *
 * Interface for WipperSnapper's GPS component hardware.
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
#include "hardware.h"

/*!
 * @brief Constructor
 */
GPSHardware::GPSHardware() {
  _iface_type = GPS_IFACE_NONE;
  _driver_type = GPS_DRV_NONE;
  _nmea_baud_rate = DEFAULT_MTK_NMEA_BAUD_RATE;
  _nmea_update_rate = DEFAULT_MTK_NMEA_UPDATE_RATE;
}

/*!
 * @brief Destructor
 */
GPSHardware::~GPSHardware() {
  // Clean up GPS instance
  if (_ada_gps) {
    delete _ada_gps;
    _ada_gps = nullptr;
  }
}

/*!
 * @brief Handles a GPSConfig message from the protobuf stream.
 * @param stream
 *        Pointer to a pb_istream_t object.
 * @returns True if the message was handled successfully, False otherwise.
 */
bool GPSHardware::Handle_GPSConfig(wippersnapper_gps_GPSConfig *gps_config) {
  WS_DEBUG_PRINTLN("[gps] Handling GPSConfig message...");
  if (gps_config == nullptr)
    return false;
  // Set the polling period for GPS data
  SetPollPeriod(gps_config->period);

  // Attempt to decode the GPSConfig message
  if (_driver_type == GPS_DRV_MTK) {
    WS_DEBUG_PRINTLN("[gps] Handling GPSConfig for MediaTek driver...");
    if (gps_config == nullptr) {
      WS_DEBUG_PRINTLN("[gps] ERROR: No GPSConfig message found!");
      return false;
    }
    // Iterate through the command sentences and send them to the GPS module
    // TODO: We may want to break this out into a generic function that supports
    // MTK, Ublox, etc...
    for (size_t i = 0; i < gps_config->commands_count; i++) {
      WS_DEBUG_PRINT("[gps] Sending command to MediaTek GPS: ");
      WS_DEBUG_PRINTLN(gps_config->commands[i]);
      // Build the PMTK ACK response for the command
      char msg_resp[MAX_NEMA_SENTENCE_LEN];
      if (!BuildPmtkAck(gps_config->commands[i], msg_resp)) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to build PMTK ACK response!");
        return false;
      }
      // Flush the RX/TX buffers before sending
      _hw_serial->flush();
      while (_hw_serial->available() > 0) {
        _hw_serial->read();
      }
      // Send the command to the GPS module
      _ada_gps->sendCommand(gps_config->commands[i]);
      // and wait for the corresponding response from the GPS module
      if (!_ada_gps->waitForSentence(msg_resp)) {
        WS_DEBUG_PRINT("[gps] ERROR: Failed to get response | cmd:");
        WS_DEBUG_PRINTLN(gps_config->commands[i]);
        return false;
      }
    }
  } else {
    WS_DEBUG_PRINTLN("[gps] ERROR: Unsupported GPS driver type!");
    return false;
  }

  return true;
}

/*!
 * @brief Sets a UART hardware interface for the GPS controller.
 * @param serial
 *        Pointer to a HardwareSerial instance.
 * @returns True if the interface was set successfully, False otherwise.
 */
bool GPSHardware::SetInterface(HardwareSerial *serial) {
  if (serial == nullptr)
    return false;
  // Configure the hardware serial interface
  _hw_serial = serial;
  _iface_type = GPS_IFACE_UART_HW;
  return true;
}

/*!
 * @brief Sets a TwoWire (I2C) interface for the GPS controller.
 * @param wire
 *        Points to a TwoWire instance.
 * @returns True if the interface was set successfully, False otherwise.
 */
bool GPSHardware::SetInterface(TwoWire *wire) {
  if (wire == nullptr)
    return false;
  // Configure the I2C interface
  _wire = wire;
  _iface_type = GPS_IFACE_I2C;
  return true;
}

/*!
 * @brief Attempts to initialize the GPS device based on the configured
 * interface type.
 * @returns True if the GPS device was initialized successfully, False
 * otherwise.
 */
bool GPSHardware::begin() {
  if (_iface_type == GPS_IFACE_NONE) {
    WS_DEBUG_PRINTLN("[gps] ERROR: No interface configured for GPS!");
    return false;
  }

  // Attempt to set the GPS interface type based on the hardware
  if (!QueryModuleType()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to query GPS module type!");
    return false;
  }
  WS_DEBUG_PRINTLN("[gps] Module detected, ready for commands!");

  return true;
}

/*!
 * @brief Queries the GPS driver type by attempting to detect MediaTek or u-blox
 * GPS modules.
 * @returns True if the driver type was detected successfully, False otherwise.
 */
bool GPSHardware::QueryModuleType() {
  WS_DEBUG_PRINTLN("[gps] Attempting to detect GPS module type...");

  if (_iface_type == GPS_IFACE_UART_HW) {
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

    WS_DEBUG_PRINTLN(
        "[gps] ERROR: Failed to detect GPS driver type, attempting "
        "to use generic driver!");
    // TODO: Implement generic NMEA GPS driver detection

    // No responses from the GPS module over the defined iface, so we bail out
    WS_DEBUG_PRINTLN("[gps] ERROR: No GPS driver type detected!");
  } else if (_iface_type == GPS_IFACE_I2C) {
    if (_addr == PA1010D_I2C_ADDRESS) {
      WS_DEBUG_PRINT("[gps] Attempting to use PA1010D driver...");
      // Attempt to use Adafruit_GPS I2c interface
      _ada_gps = new Adafruit_GPS(_hw_serial);
      if (!_ada_gps->begin(_addr)) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize Mediatek!");
        return false;
      }
      WS_DEBUG_PRINTLN("ok!");
      _driver_type = GPS_DRV_MTK;
      return true;
    } else {
      WS_DEBUG_PRINTLN(
          "[gps] ERROR: Only PA1010D i2c module is supported at this time!");
      return false;
    }
  }

  return false;
}

/*!
 * @brief Detects if the GPS module is a MediaTek GPS module by querying its
 * firmware version.
 * @returns True if a MediaTek GPS module was detected, False otherwise.
 */
bool GPSHardware::DetectMediatek() {
  if (_iface_type != GPS_IFACE_UART_HW) {
    WS_DEBUG_PRINTLN("[gps] ERROR: MediaTek GPS module only supports Hardware "
                     "Serial interface!");
    return false;
  }

  // Clear the tx and rx buffers before sending the command
  _hw_serial->flush();
  while (_hw_serial->available() > 0) {
    _hw_serial->read();
  }
  _hw_serial->println(CMD_MTK_QUERY_FW);
  // Query MediaTek firmware version
  // Wait for response
  uint16_t timeout = 2000; // 1 second timeout
  while (_hw_serial->available() < MAX_NEMA_SENTENCE_LEN && timeout--) {
    delay(1);
  }

  if (timeout == 0)
    return false;

  // We found a response, let's verify that it's the expected PMTK_DK_RELEASE
  // command by reading out the NMEA sentence string into a buffer
  size_t buf_len = MAX_NEMA_SENTENCE_LEN * 4; // +3 for \r\n and null terminator
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
  _ada_gps = new Adafruit_GPS(_hw_serial);
  if (!_ada_gps->begin(_hw_serial->baudRate())) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize Mediatek!");
    return false;
  }
  _driver_type = GPS_DRV_MTK;
  return true;
}

/*!
 * @brief Builds a PMTK acknowledgment message for the provided command.
 * @param msg_cmd
 *        Pointer to a command string.
 * @param msg_resp
 *        Pointer to a response string.
 * @returns True if the acknowledgment was built successfully, False otherwise.
 */
bool GPSHardware::BuildPmtkAck(char *msg_cmd, char *msg_resp) {
  int cmd_num = 0;
  if (sscanf(msg_cmd, "$PMTK%d", &cmd_num) != 1)
    return false;
  snprintf(msg_resp, MAX_NEMA_SENTENCE_LEN, "$PMTK001,%d,3", cmd_num);
  _ada_gps->addChecksum(msg_resp);
  return true;
}

/*!
 * @brief Sets the polling period for GPS data.
 * @param poll_period
 *        The polling period in milliseconds.
 */
void GPSHardware::SetPollPeriod(ulong poll_period) {
  if (poll_period < 0) {
    _period = 0;
    return;
  }
  _period = (unsigned long)(poll_period * 1000.0f);
}

/*!
 * @brief Sets the previous polling period for GPS data.
 * @param poll_period_prv
 *        The previous polling period in milliseconds.
 */
void GPSHardware::SetPollPeriodPrv(ulong poll_period_prv) {
  _period_prv = poll_period_prv;
}

/*!
 * @brief Gets the current polling period for GPS data.
 * @returns The polling period in milliseconds.
 */
ulong GPSHardware::GetPollPeriod() { return _period; }

/*!
 * @brief Gets the previous polling period for GPS data.
 * @returns The previous polling period in milliseconds.
 */
ulong GPSHardware::GetPollPeriodPrv() { return _period_prv; }

/*!
 * @brief Returns the Adafruit_GPS instance.
 * @returns Pointer to the Adafruit_GPS instance.
 */
Adafruit_GPS *GPSHardware::GetAdaGps() { return _ada_gps; }

/*!
 * @brief Sets the NMEA update rate for GPS data.
 * @param nmea_update_rate
 *        The NMEA update rate, in Hz.
 */
void GPSHardware::SetNmeaUpdateRate(int nmea_update_rate) {
  _nmea_update_rate = nmea_update_rate;
}

/*!
 * @brief Returns the NMEA port update rate for GPS data.
 * @returns The NMEA update rate, in Hz.
 */
int GPSHardware::GetNmeaUpdateRate() { return _nmea_update_rate; }

/*!
 * @brief Sets the NMEA baud rate for GPS data.
 * @param nmea_baud_rate
 *        The NMEA baud rate, in bits per second.
 */
void GPSHardware::SetNmeaBaudRate(int nmea_baud_rate) {
  _nmea_baud_rate = nmea_baud_rate;
}

/*!
 * @brief Returns the NMEA port baud rate for GPS data.
 * @returns The NMEA baud rate, in bits per second.
 */
int GPSHardware::GetNmeaBaudRate() { return _nmea_baud_rate; }

/*!
 * @brief Sets the I2C address for a GPS.
 * @param i2c_address
 *       The I2C address to set for the GPS device.
 */
void GPSHardware::SetI2CAddress(uint32_t i2c_address) { _addr = i2c_address; }

/*!
 * @brief Sets the last time the GPS hardware was polled.
 * @param kat_prv
 *        The last time the GPS hardware was polled, in milliseconds.
 */
void GPSHardware::SetPrvKat(ulong kat_prv) {
  if (kat_prv < 0) {
    _kat_prv = 0;
    return;
  }
  _kat_prv = kat_prv;
}

/*!
 * @brief   Gets the last time the GPS hardware was polled.
 * @returns The last time the GPS hardware was polled, in milliseconds.
 */
ulong GPSHardware::GetPrvKat() { return _kat_prv; }

/*!
 * @brief Returns the driver type of the GPS hardware.
 * @returns The driver type of the GPS hardware.
 */
GpsDriverType GPSHardware::GetDriverType() { return _driver_type; }