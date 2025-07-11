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
  _iface_type = GPS_IFACE_NONE;
  _driver_type = GPS_DRV_NONE;
  if (_ada_gps) {
    delete _ada_gps;
    _ada_gps = nullptr;
  }
  if (_sfe_gps) {
    delete _sfe_gps;
    _sfe_gps = nullptr;
  }
}

/*!
 * @brief Helper function that reads and discards data requested by the I2C bus.
 */
void GPSHardware::I2cReadDiscard() {
  // TODO: Let's hope this works for UBX too! Remove if its ok
  _wire->flush();
  uint8_t bytes_requested = 32;
  _wire->requestFrom(_addr, bytes_requested);
  while (_wire->available()) {
    _wire->read();
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
  // Set the module's polling period for GPS data
  SetPollPeriod(gps_config->period);

  // Attempt to decode the GPSConfig message
  if (_driver_type == GPS_DRV_MTK) {
    WS_DEBUG_PRINTLN("[gps] Handling GPSConfig for MediaTek driver...");
    // Iterate through the command sentences and send them to the GPS module
    for (size_t i = 0; i < gps_config->commands_pmtks_count; i++) {
      // Build the PMTK ACK response for the command
      char msg_resp[MAX_NEMA_SENTENCE_LEN];
      if (!BuildPmtkAck(gps_config->commands_pmtks[i], msg_resp)) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to build PMTK ACK response!");
        return false;
      }
      WS_DEBUG_PRINT("[gps] Expected response: ");
      WS_DEBUG_PRINTLN(msg_resp);
      if (_iface_type == GPS_IFACE_UART_HW) {
        // Flush the RX/TX buffers before sending
        _hw_serial->flush();
        while (_hw_serial->available() > 0) {
          _hw_serial->read();
        }
      } else if (_iface_type == GPS_IFACE_I2C) {
        WS_DEBUG_PRINT("[gps] Flushing I2C buffers...");
        I2cReadDiscard();
        WS_DEBUG_PRINTLN(" done!");
      }
      WS_DEBUG_PRINT("[gps] TX, CMD: ");
      WS_DEBUG_PRINTLN(gps_config->commands_pmtks[i]);
      // Send the command to the GPS module
      _ada_gps->sendCommand(gps_config->commands_pmtks[i]);
      WS_DEBUG_PRINTLN("[gps] Waiting for RX...");
      // and wait for the corresponding response from the GPS module
      if (!_ada_gps->waitForSentence(msg_resp, 255)) {
        WS_DEBUG_PRINT("[gps] ERROR: Failed to get response | cmd:");
        WS_DEBUG_PRINTLN(gps_config->commands_pmtks[i]);
        return false;
      }
    }
  } else if (_driver_type == GPS_DRV_UBLOX) {
    WS_DEBUG_PRINTLN("[gps] Handling GPSConfig for U-Blox driver...");
    // Iterate through the command sentences and send them to the GPS module
    for (size_t i = 0; i < gps_config->commands_ubxes_count; i++) {
      I2cReadDiscard();
      // Write this to the GPS module
      // TODO - We are just printing here to debug
      WS_DEBUG_PRINT("[gps] Sending UBX CMD #: ");
      WS_DEBUG_PRINTLN(i);
      WS_DEBUG_PRINT("\tUBX CMDSZ: ");
      WS_DEBUG_PRINTLN(gps_config->commands_ubxes[i].size);
      WS_DEBUG_PRINT("\tUBX CMD: ");
      for (pb_size_t j = 0; j < gps_config->commands_ubxes[i].size; j++) {
        WS_DEBUG_PRINT(gps_config->commands_ubxes[i].bytes[j], HEX);
        WS_DEBUG_PRINT(" ");
      }
      if (!_sfe_gps->pushRawData(gps_config->commands_ubxes[i].bytes,
                                 gps_config->commands_ubxes[i].size)) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to push bytes to module!");
        return false;
      }
      WS_DEBUG_PRINTLN("Sent!");
      // Wait for the ACK response from the GPS module
      // Build a dummy packet for the ACK response
      uint8_t payloadAck[2];
      ubxPacket packetAck = {0,
                             0,
                             0,
                             0,
                             0,
                             payloadAck,
                             0,
                             0,
                             SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
                             SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};
      uint8_t cmdClass = gps_config->commands_ubxes[i].bytes[2];
      uint8_t cmdId = gps_config->commands_ubxes[i].bytes[3];

      // Wait for ACK response with timeout
      sfe_ublox_status_e rc =
          _sfe_gps->waitForACKResponse(&packetAck, cmdClass, cmdId, 1000);
      WS_DEBUG_PRINT("waitForACKResponse res: ");
      WS_DEBUG_PRINTLN(rc);
      if (rc == SFE_UBLOX_STATUS_DATA_RECEIVED ||
          rc == SFE_UBLOX_STATUS_DATA_SENT) {
        WS_DEBUG_PRINTLN("UBX RX'd!");
        // TODO: We don't need this, we can just check rc != above instead of ==
        // and handle that case only
      } else {
        WS_DEBUG_PRINTLN("UBX timeout or error!");
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
  WS_DEBUG_PRINTLN("[gps] Module detected and ready!");
  return true;
}

/*!
 * @brief Attempts to detect and initialize a MediaTek GPS module over I2C
 * @returns True if a MediaTek GPS module was detected and initialized, False
 * otherwise.
 */
bool GPSHardware::DetectMtkI2C(uint32_t addr) {
  if (_addr != PA1010D_I2C_ADDRESS) {
    WS_DEBUG_PRINTLN(
        "[gps] ERROR: Only PA1010D i2c module is supported at this time!");
    return false;
  }
  _ada_gps = new Adafruit_GPS(_wire);
  if (!_ada_gps->begin(_addr))
    return false;
  _driver_type = GPS_DRV_MTK;
  return true;
}

/*!
 * @brief Attempts to detect and initialize a U-Blox GPS module over I2C
 * @param addr
 *        The I2C address of the GPS module.
 * @returns True if a u-blox GPS module was detected and initialized, False
 * otherwise.
 */
bool GPSHardware::DetectUbxI2C(uint32_t addr) {
  if (addr != UBX_I2C_ADDRESS) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Invalid U-Blox I2C address!");
    return false;
  }
  _sfe_gps = new SFE_UBLOX_GNSS();
  if (!_sfe_gps->begin(*_wire, _addr))
    return false;
  // Configure the u-blox GPS module
  _sfe_gps->setI2COutput(
      COM_TYPE_UBX |
      COM_TYPE_NMEA); // Set the I2C port to output both NMEA and UBX messages
  _sfe_gps->saveConfigSelective(
      VAL_CFG_SUBSEC_IOPORT); // Save (only) the communications port settings to
                              // flash and BBR
  _sfe_gps->setProcessNMEAMask(
      SFE_UBLOX_FILTER_NMEA_ALL); // Make sure the library is passing all NMEA
                                  // messages to processNMEA
  _sfe_gps->setProcessNMEAMask(
      SFE_UBLOX_FILTER_NMEA_GGA); // Or, we can be kind to MicroNMEA and _only_
                                  // pass the GGA messages to it
  _driver_type = GPS_DRV_UBLOX;
  return true;
}

/*!
 * @brief Queries the GPS driver type by attempting to detect MediaTek or u-blox
 * GPS modules.
 * @returns True if the driver type was detected successfully, False otherwise.
 */
bool GPSHardware::QueryModuleType() {
  WS_DEBUG_PRINTLN("[gps] Detecting module type...");
  if (_iface_type == GPS_IFACE_UART_HW) {
    // Try to detect MediaTek GPS module
    if (DetectMtkUart()) {
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
      if (!DetectMtkI2C(_addr)) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to init PA1010D module!");
        return false;
      }
      WS_DEBUG_PRINTLN("ok!");
      return true;
    } else if (_addr == UBX_I2C_ADDRESS) {
      WS_DEBUG_PRINT("[gps] Attempting to use u-blox driver...");
      if (!DetectUbxI2C(_addr)) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to init u-blox module!");
        return false;
      }
      WS_DEBUG_PRINTLN("ok!");
      return true;
    } else {
      WS_DEBUG_PRINTLN("[gps] ERROR: Uknown I2C address provided!");
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
bool GPSHardware::DetectMtkUart() {
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
 * @brief Returns the SFE_UBLOX_GNSS instance.
 * @returns Pointer to the SFE_UBLOX_GNSS instance.
 */
SFE_UBLOX_GNSS *GPSHardware::GetUbxGps() { return _sfe_gps; }

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

/*!
 * @brief Returns the interface type of the GPS hardware.
 * @returns The interface type of the GPS hardware.
 */
GpsInterfaceType GPSHardware::GetIfaceType() { return _iface_type; }

/*!
 * @brief Discards any data in the UART RX buffer.
 */
void GPSHardware::UartReadDiscard() {
  if (_driver_type == GPS_DRV_MTK) {
    size_t bytes_avail = _ada_gps->available();
    if (bytes_avail > 0) {
      for (size_t i = 0; i < bytes_avail; i++) {
        _ada_gps->read();
      }
    }
  }
  // TODO: Support UBX's UART iface here
}

/*!
 * @brief Discards any data in the UART or I2C RX buffer.
 */
void GPSHardware::ReadDiscardBuffer() {
  if (_iface_type == GPS_IFACE_UART_HW) {
    UartReadDiscard();
  } else if (_iface_type == GPS_IFACE_I2C) {
    I2cReadDiscard();
  } else {
    WS_DEBUG_PRINTLN("[gps] ERROR: Unsupported GPS interface type!");
  }
}

/*!
 * @brief Polls the GPS hardware for new NMEA sentences and stores them in a
 * circular buffer.
 */
void GPSHardware::PollStoreSentences() {
  if (_driver_type == GPS_DRV_MTK) {
    // Before we poll, Unset the RX flag
    if (_ada_gps->newNMEAreceived())
      _ada_gps->lastNMEA();

    // Read from the GPS module for update_rate milliseconds
    ulong update_rate = 1000 / _nmea_update_rate;
    ulong start_time = millis();
    while (millis() - start_time < update_rate) {
      char c = _ada_gps->read();
      // Check if we have a new NMEA sentence
      if (_ada_gps->newNMEAreceived()) {
        // If we have a new sentence, push it to the buffer
        char *last_nmea = _ada_gps->lastNMEA();
        NmeaBufPush(_ada_gps->lastNMEA());
      }
    }
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO!
  } else {
    WS_DEBUG_PRINTLN("[gps] ERROR: Unsupported GPS driver type for polling!");
  }
}

/*!
 * @brief Pushes a new NMEA sentence into the circular buffer.
 * @param new_sentence Pointer to the new NMEA sentence to be added.
 * @return 0 on success, -1 if the buffer is full.
 */
int GPSHardware::NmeaBufPush(const char *new_sentence) {
  if (!new_sentence)
    return -1;

  int next = _nmea_buff.head + 1; // points to head after the current write
  if (next >= _nmea_buff.maxlen)
    next = 0; // wrap around

  // If buffer is full, advance tail to overwrite oldest data
  if (next == _nmea_buff.tail) {
    _nmea_buff.tail = (_nmea_buff.tail + 1) % _nmea_buff.maxlen;
  }

  // Copy the new sentence into the buffer
  strncpy(_nmea_buff.sentences[_nmea_buff.head], new_sentence,
          MAX_LEN_NMEA_SENTENCE - 1);
  _nmea_buff.sentences[_nmea_buff.head][MAX_LEN_NMEA_SENTENCE - 1] = '\0';
  _nmea_buff.head = next;
  return 0;
}

/*!
 * @brief Pops a NMEA sentence from the circular buffer, FIFO order.
 * @param sentence Pointer to the buffer where the popped sentence will be
 * stored.
 * @return 0 on success, -1 if the buffer is empty.
 */
int GPSHardware::NmeaBufPop(char *sentence) {
  // Is the buffer empty?
  if (_nmea_buff.head == _nmea_buff.tail)
    return -1;

  int next =
      _nmea_buff.tail + 1; // next is where tail will point to after this read.
  if (next >= _nmea_buff.maxlen)
    next = 0;

  // Copy sentence from tail
  strcpy(sentence, _nmea_buff.sentences[_nmea_buff.tail]);
  _nmea_buff.tail = next; // Advance tail
  return 0;
}

/*!
 * @brief Parses a NMEA sentence and returns true if it was successfully parsed.
 * @param sentence Pointer to the NMEA sentence to be parsed.
 * @returns True if the sentence was parsed successfully, False otherwise.
 */
bool GPSHardware::ParseNMEASentence(char *sentence) {
  if (!sentence)
    return false;
  if (_driver_type == GPS_DRV_MTK) {
    // Parse the NMEA sentence using Adafruit_GPS
    return _ada_gps->parse(sentence);
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // Parse the NMEA sentence using SFE_UBLOX_GNSS
    // TODO!
    // return _sfe_gps->parseNMEA(sentence);
  }

  return false;
}

uint8_t GPSHardware::GetHour() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->hour;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}

uint8_t GPSHardware::GetMinute() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->minute;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}

uint8_t GPSHardware::GetSeconds() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->seconds;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}

uint16_t GPSHardware::GetMilliseconds() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->milliseconds;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}

uint8_t GPSHardware::GetDay() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->day;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}

uint8_t GPSHardware::GetMonth() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->month;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}

uint8_t GPSHardware::GetYear() {
  if (_driver_type == GPS_DRV_MTK) {
    return _ada_gps->year;
  } else if (_driver_type == GPS_DRV_UBLOX) {
    // TODO: Implement for UBLOX
  }
  return 0;
}