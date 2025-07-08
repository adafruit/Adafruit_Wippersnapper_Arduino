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
 * @brief Constructor for GPSController.
 */
GPSController::GPSController() {
  _gps_model = new GPSModel();
  _nmea_buff.head = 0;
  _nmea_buff.tail = 0;
  _nmea_buff.maxlen = MAX_NMEA_SENTENCES;
}

/*!
 * @brief Destructor for GPSController.
 */
GPSController::~GPSController() {
  // Clean up model
  if (_gps_model) {
    delete _gps_model;
    _gps_model = nullptr;
  }
}

/*!
 * @brief Adds an I2C GPS hardware instance to the controller.
 * @param wire Pointer to the TwoWire instance for I2C communication.
 * @param i2c_addr I2C address of the GPS device.
 * @param gps_config Pointer to the GPS configuration message.
 * @return True if the GPS was added successfully, false otherwise.
 */
bool GPSController::AddGPS(TwoWire *wire, uint32_t i2c_addr,
                           wippersnapper_gps_GPSConfig *gps_config) {
  GPSHardware *gps_hw = new GPSHardware();

  if (!gps_hw->SetInterface(wire)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to set module interface!");
    delete gps_hw;
    return false;
  }

  gps_hw->SetI2CAddress(i2c_addr);
  if (!gps_hw->begin()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize module!");
    delete gps_hw;
    return false;
  }

  if (!gps_hw->Handle_GPSConfig(gps_config)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Configuration failed!");
    delete gps_hw;
    return false;
  }

  _gps_drivers.push_back(gps_hw);
  WS_DEBUG_PRINTLN("[gps] GPS hardware added successfully!");
  return true;
}

/*!
 * @brief Adds a GPS hardware instance to the controller.
 * @param serial Pointer to the HardwareSerial instance for GPS communication.
 * @param gps_config Pointer to the GPS configuration message.
 * @return True if the GPS was added successfully, false otherwise.
 */
bool GPSController::AddGPS(HardwareSerial *serial,
                           wippersnapper_gps_GPSConfig *gps_config) {
  GPSHardware *gps_hw = new GPSHardware();

  if (!gps_hw->SetInterface(serial)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to set GPS UART interface!");
    delete gps_hw;
    return false;
  }

  if (!gps_hw->begin()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize GPS hardware!");
    delete gps_hw;
    return false;
  }
  // Required - let the GPS spit out its initial data
  delay(1000);

  if (!gps_hw->Handle_GPSConfig(gps_config)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to configure GPS!");
    delete gps_hw;
    return false;
  }

  _gps_drivers.push_back(gps_hw);
  WS_DEBUG_PRINTLN("[gps] GPS hardware added successfully!");
  return true;
}

/*!
 * @brief Pushes a new NMEA sentence into the circular buffer.
 * @param new_sentence Pointer to the new NMEA sentence to be added.
 * @return 0 on success, -1 if the buffer is full.
 */
int GPSController::NmeaBufPush(const char *new_sentence) {
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
int GPSController::NmeaBufPop(char *sentence) {
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
 * @brief Updates the GPSController, polling the GPS hardware for data.
 * This function checks if the read period has elapsed and processes the GPS
 * data accordingly.
 */
void GPSController::update() {
  if (_gps_drivers.empty())
    return; // bail-out!

  for (GPSHardware *drv : _gps_drivers) {
    // Get the GPS driver interface from the hardware instance
    Adafruit_GPS *ada_gps = nullptr;
    if (drv->GetDriverType() == GPS_DRV_MTK) {
      // Interface shouldn't matter here because we already set it up in the
      // initialization phase, so we can just grab the Adafruit_GPS instance
      ada_gps = drv->GetAdaGps();
      if (ada_gps == nullptr) {
        WS_DEBUG_PRINTLN(
            "[gps] ERROR: Can't read - GPS instance not initialized!");
        continue;
      }
    } else {
      WS_DEBUG_PRINTLN(
          "[gps] ERROR: Unsupported GPS driver type, skipping update()!");
      continue;
    }

    // TODO: Commented out due to parsing failures, stability issue (failed to
    // parse NMEA acks for this) Perform a keep-alive check by sending an
    // antenna check command every 2 seconds
    /*     if (millis() - drv->GetPrvKat() > 2000) {
          drv->GetAdaGps()->sendCommand(CMD_MTK_CHECK_ANTENNA);
          drv->SetPrvKat(millis());
        } */

    // TODO: Did the polling periods get set? Let's print and check them
    WS_DEBUG_PRINT("[gps] Poll period: ");
    WS_DEBUG_PRINT(drv->GetPollPeriod());
    WS_DEBUG_PRINT(", Previous poll period: ");
    WS_DEBUG_PRINTLN(drv->GetPollPeriodPrv());

    // Did read period elapse?
    ulong cur_time = millis();
    if (cur_time - drv->GetPollPeriodPrv() < drv->GetPollPeriod())
      continue; // Not yet elapsed, skip this driver

    // Discard the GPS buffer before we attempt to do a fresh read
    WS_DEBUG_PRINTLN("[gps] Discarding GPS buffer...");
    WS_DEBUG_PRINT("iface type: ");
    WS_DEBUG_PRINTLN(drv->GetIfaceType());

    if (drv->GetIfaceType() == GPS_IFACE_UART_HW) {
      // TODO: Refactor this into a function within hardware.cpp
      size_t bytes_avail = ada_gps->available();
      if (bytes_avail > 0) {
        for (size_t i = 0; i < bytes_avail; i++) {
          WS_DEBUG_PRINT("[gps] Reading byte: ");
          WS_DEBUG_PRINT(i);
          ada_gps->read();
          WS_DEBUG_PRINTLN("...OK!");
        }
      }
    } else if (drv->GetIfaceType() == GPS_IFACE_I2C) {
      // For I2C, request and discard any stale data from the device
      WS_DEBUG_PRINTLN("[gps] Discarding stale I2C data...");
      drv->I2cReadDiscard();
    }

    // if (drv->GetIfaceType() == GPS_IFACE_UART_HW) {

      // Unset the RX flag
      WS_DEBUG_PRINT("[gps] Unsetting RX flag...");
      if (ada_gps->newNMEAreceived()) {
        ada_gps->lastNMEA();
      }
      WS_DEBUG_PRINT("ok");

      // Let's attempt to get a sentence from the GPS module
      // Read from the GPS module for update_rate milliseconds
      WS_DEBUG_PRINT("[gps] GetNmeaUpdateRate...");
      ulong update_rate = 1000 / drv->GetNmeaUpdateRate();
      ulong start_time = millis();
      WS_DEBUG_PRINT("ok");

      WS_DEBUG_PRINT("[gps] Reading GPS data for ");
      WS_DEBUG_PRINT(update_rate);
      WS_DEBUG_PRINTLN(" ms...");
      while (millis() - start_time < update_rate) {
        char c = ada_gps->read();
        // Check if we have a new NMEA sentence
        if (ada_gps->newNMEAreceived()) {
          // If we have a new sentence, push it to the buffer
          char *last_nmea = ada_gps->lastNMEA();
          NmeaBufPush(ada_gps->lastNMEA());
        }
      }

      // Parse each NMEA sentence in the buffer
      char nmea_sentence[MAX_LEN_NMEA_SENTENCE];
      bool has_gps_event = false;
      while (NmeaBufPop(nmea_sentence) != -1) {
        // Parse the NMEA sentence
        WS_DEBUG_PRINT("[gps] Parsing NMEA sentence: ");
        WS_DEBUG_PRINTLN(nmea_sentence);
        if (!ada_gps->parse(nmea_sentence)) {
          continue; // Skip parsing this sentence if parsing failed
        } else {
          _gps_model->CreateGPSEvent();
          has_gps_event = true;
        }

        // Build the GPSEvent message from the sentence
        wippersnapper_gps_GPSDateTime datetime = _gps_model->CreateGpsDatetime(
            ada_gps->hour, ada_gps->minute, ada_gps->seconds,
            ada_gps->milliseconds, ada_gps->day, ada_gps->month, ada_gps->year);
        if (strncmp(nmea_sentence, "$GPRMC", 6) == 0) {
          _gps_model->AddGpsEventRMC(datetime, ada_gps->fix, ada_gps->latitude,
                                     &ada_gps->lat, ada_gps->longitude,
                                     &ada_gps->lon, ada_gps->speed,
                                     ada_gps->angle);
        } else if (strncmp(nmea_sentence, "$GPGGA", 6) == 0) {
          _gps_model->AddGpsEventGGA(
              datetime, ada_gps->fix, ada_gps->latitude, &ada_gps->lat,
              ada_gps->longitude, &ada_gps->lon, ada_gps->satellites,
              ada_gps->HDOP, ada_gps->altitude, ada_gps->geoidheight);
        } else {
          WS_DEBUG_PRINTLN(
              "[gps] WARNING - Parsed sentence is not type RMC or GGA!");
        }
      }

      // We did not create a GPSEvent because the NMEA sentences were not
      // GGA/RMC or parsed correctly
      if (!has_gps_event) {
        WS_DEBUG_PRINTLN("[gps] No GPSEvent created from NMEA sentences!");
        continue;
      }

      // Encode and publish to IO
      WS_DEBUG_PRINT("[gps] Encoding and publishing GPSEvent to IO...");
      bool did_encode = _gps_model->EncodeGPSEvent();
      if (!did_encode) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to encode GPSEvent!");
      } else {
        // Publish the GPSEvent to IO
        if (!WsV2.PublishSignal(
                wippersnapper_signal_DeviceToBroker_gps_event_tag,
                _gps_model->GetGPSEvent())) {
          WS_DEBUG_PRINTLN("[gps] ERROR: Failed to publish GPSEvent!");
        } else {
          WS_DEBUG_PRINTLN("[gps] GPSEvent published successfully!");
        }
      }
      drv->SetPollPeriodPrv(cur_time);
  }
}