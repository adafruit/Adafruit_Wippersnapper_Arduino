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

  // Initialize NMEA buffer
  _nmea_buff.head = 0;
  _nmea_buff.tail = 0;
  _nmea_buff.maxlen = MAX_NMEA_SENTENCES;
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
 * @param gps_config
 *        Pointer to a wippersnapper_gps_GPSConfig message.
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
    I2cReadDiscard();
    // Iterate through the command sentences and send them to the GPS module
    for (size_t i = 0; i < gps_config->commands_ubxes_count; i++) {
      // TODO: Tuesday fix this frame decoder!
      /*       pb_bytes_array_t *ubx_frame = &gps_config->commands_ubxes[i];
            // Validate minimum frame size
            if (ubx_frame->size < 8) {
              WS_DEBUG_PRINT("[gps] Invalid UBX frame size: ");
              WS_DEBUG_PRINTLN(ubx_frame->size);
              continue;
            }

            // Validate sync bytes
            if (ubx_frame->bytes[0] != 0xB5 || ubx_frame->bytes[1] != 0x62) {
              WS_DEBUG_PRINTLN("[gps] Invalid UBX sync bytes");
              continue;
            }

            // Validate frame size
            size_t expectedSize = 8 + payloadLength;
            if (ubx_frame->size != expectedSize) {
              WS_DEBUG_PRINT("[gps] Frame size mismatch. Expected: ");
              WS_DEBUG_PRINT(expectedSize);
              WS_DEBUG_PRINT(", Got: ");
              WS_DEBUG_PRINTLN(ubx_frame->size);
              continue;
            }

            // Extract message components
            uint8_t msgClass = ubx_frame->bytes[2];
            uint8_t msgId = ubx_frame->bytes[3];
            uint16_t payloadLength = ubx_frame->bytes[4] | (ubx_frame->bytes[5]
         << 8);

            // Get payload
            uint8_t *payload = NULL;
            if (payloadLength > 0) {
              payload = &ubx_frame->bytes[6];
            }

            WS_DEBUG_PRINT("[gps] Sending UBX CMD #");
            WS_DEBUG_PRINT(i);
            WS_DEBUG_PRINT(" - Class: 0x");
            WS_DEBUG_PRINT(msgClass, HEX);
            WS_DEBUG_PRINT(", ID: 0x");
            WS_DEBUG_PRINT(msgId, HEX);
            WS_DEBUG_PRINT(", Payload len: ");
            WS_DEBUG_PRINTLN(payloadLength);

            // Send the message
            UBXSendStatus status = _ubx_gps->sendMessageWithAck(msgClass, msgId,
         payload, payloadLength);

            if (status != UBXSendStatus::UBX_SEND_SUCCESS) {
              WS_DEBUG_PRINTLN("[gps] Failed to send UBX message");
            } else {
              WS_DEBUG_PRINTLN("[gps] OK");
            } */
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
  _ubx_gps_ddc = new Adafruit_UBloxDDC(addr, _wire);
  if (!_ubx_gps_ddc->begin())
    return false;
  _ubx_gps = new Adafruit_UBX(*_ubx_gps_ddc);
  if (!_ubx_gps->begin())
    return false;
  _ubx_gps->verbose_debug = 3; // TODO: Set this to 1 in production
  // Configure Adafruit_GPS instance for parsing NMEA sentences only
  _ada_gps = new Adafruit_GPS();
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

    WS_DEBUG_PRINTLN("[gps] Failed to detect MTK GPS module!");
    // Future TODO: Implement u-blox detection here for UART
    // Future TODO: Implement generic NMEA GPS driver detection fallback
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
  int baud_rate;
#if defined(ARDUINO_RASPBERRY_PI_PICO_W) ||                                    \
    defined(ARDUINO_RASPBERRY_PI_PICO_2W) ||                                   \
    defined(ADAFRUIT_METRO_M4_EXPRESS) ||                                      \
    defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || defined(ADAFRUIT_PYPORTAL) ||   \
    defined(ADAFRUIT_PYPORTAL_M4_TITANO)
  baud_rate = 9600; // Pico SDK does not support getting baud rate from serial,
                    // default to 9600
#else
  baud_rate = _hw_serial->baudRate();
#endif
  if (!_ada_gps->begin(baud_rate)) {
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
    // Read from the GPS module for update_rate milliseconds
    // TODO: Too many calls to WS_DEBUG
    // TOO: Maybe just delay for update_rate instead of constantly polling the
    // hw serial for avability? more performant
    ulong update_rate = 1000 / _nmea_update_rate;
    ulong start_time = millis();
    while (millis() - start_time < update_rate) {
      _ubx_gps_ddc->available();
    }
    uint8_t buffer[MAX_LEN_NMEA_SENTENCE];
    String nmeaBuffer = "";
    int bytesAvailable = _ubx_gps_ddc->available();
    size_t bytesRead;
    size_t bytesToRead = min(bytesAvailable, 82);
    if (bytesAvailable > 0) {
      bytesRead = _ubx_gps_ddc->readBytes(buffer, bytesToRead);
    }
    // Build NMEA sentences and parse when complete
    for (size_t i = 0; i < bytesRead; i++) {
      char c = buffer[i];
      nmeaBuffer += c;
      // Check for end of NMEA sentence
      if (c == '\n') {
        WS_DEBUG_PRINT("[gps] GOT NMEA sentence: ");
        WS_DEBUG_PRINTLN(nmeaBuffer.c_str());
        // Push the NMEA sentence to the buffer
        WS_DEBUG_PRINTLN("[gps] Pushing NMEA sentence to buffer...");
        if (NmeaBufPush(nmeaBuffer.c_str()) != 0) {
          WS_DEBUG_PRINTLN(
              "[gps] ERROR: Unable to push NMEA sentence to buffer!");
        } else {
          nmeaBuffer = ""; // Reset buffer for next sentence
        }
      }
    }
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

  // If buffer is full, overwrite oldest data
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
  return _ada_gps->parse(sentence);
}

/*!
 * @brief Gets the hours from the GPS module.
 * @returns The hours (0-23), or 0 if the GPS driver type is not set.
 */
uint8_t GPSHardware::GetHour() { return _ada_gps->hour; }

/*!
 * @brief Gets the minute from the GPS module.
 * @returns The minute (0-59), or 0 if the GPS driver type is not set.
 */
uint8_t GPSHardware::GetMinute() { return _ada_gps->minute; }

/*!
 * @brief Gets the seconds from the GPS module.
 * @returns The seconds (0-59), or 0 if the GPS driver type is not set.
 */
uint8_t GPSHardware::GetSeconds() { return _ada_gps->seconds; }

/*!
 * @brief Gets the milliseconds from the GPS module.
 * @returns The milliseconds part of the current time, or 0 if the GPS driver
 * type is not set.
 */
uint16_t GPSHardware::GetMilliseconds() { return _ada_gps->milliseconds; }

/*!
 * @brief Gets the day from the GPS module.
 * @returns The day of the month (1-31), or 0 if the GPS driver type is not set.
 */
uint8_t GPSHardware::GetDay() { return _ada_gps->day; }

/*!
 * @brief Gets the month from the GPS module.
 * @returns The month as a number (1-12), or 0 if the GPS driver type is not
 * set.
 */
uint8_t GPSHardware::GetMonth() { return _ada_gps->month; }

/*!
 * @brief Gets the year from the GPS module.
 * @returns The year as a 2-digit number (e.g., 23 for 2023), or 0 if the GPS
 * driver type is not set.
 */
uint8_t GPSHardware::GetYear() { return _ada_gps->year; }

/*!
 * @brief Gets the GPS fix status.
 * @returns True if the GPS has a fix, False otherwise.
 */
bool GPSHardware::GetFix() { return _ada_gps->fix; }

/*!
 * @brief Gets the GPS latitude.
 * @returns The latitude in degrees, or 0.0 if the GPS driver type is not set.
 */
float GPSHardware::GetLat() { return _ada_gps->latitude; }

/*!
 * @brief Gets the GPS latitude direction.
 * @returns The latitude direction as a character ('N' or 'S'), or '\0' if the
 * GPS driver type is not set.
 */
char GPSHardware::GetLatDir() { return _ada_gps->lat; }

/*!
 * @brief Gets the GPS longitude.
 * @returns The longitude in degrees, or 0.0 if the GPS driver type is not set.
 */
float GPSHardware::GetLon() { return _ada_gps->longitude; }

/*!
 * @brief Gets the GPS longitude direction.
 * @returns The longitude direction as a character ('E' or 'W'), or '\0' if the
 * GPS driver type is not set.
 */
char GPSHardware::GetLonDir() { return _ada_gps->lon; }

/*!
 * @brief Gets the number of satellites in view.
 * @returns The number of satellites in view, or 0 if the GPS driver type is
 * not set.
 */
uint8_t GPSHardware::GetNumSats() { return _ada_gps->satellites; }

/*!
 * @brief Gets the horizontal dilution of precision (HDOP).
 * @returns The HDOP value, or 0.0 if the GPS driver type is not set.
 */
float GPSHardware::GetHDOP() { return _ada_gps->HDOP; }

/*!
 * @brief Gets the altitude from the GPS module.
 * @returns The altitude in meters, or 0.0 if the GPS driver type is not set.
 */
float GPSHardware::GetAltitude() { return _ada_gps->altitude; }

/*!
 * @brief Gets the speed from the GPS module.
 * @returns The speed in meters per second, or 0.0 if the GPS driver type is
 * not set.
 */
float GPSHardware::GetSpeed() { return _ada_gps->speed; }

/*!
 * @brief Gets the angle from the GPS module.
 * @returns The angle in degrees, or 0.0 if the GPS driver type is not set.
 */
float GPSHardware::GetAngle() { return _ada_gps->angle; }

/*!
 * @brief Gets the geoid height from the GPS module.
 * @returns The geoid height in meters, or 0.0 if the GPS driver type is not
 * set.
 */
float GPSHardware::GetGeoidHeight() { return _ada_gps->geoidheight; }