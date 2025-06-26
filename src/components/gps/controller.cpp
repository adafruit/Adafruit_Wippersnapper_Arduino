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
 * @brief Adds a GPS hardware instance to the controller.
 * @param serial Pointer to the HardwareSerial instance for GPS communication.
 * @param gps_config Pointer to the GPS configuration message.
 * @return True if the GPS was added successfully, false otherwise.
 */
bool GPSController::AddGPS(HardwareSerial *serial,
                           wippersnapper_gps_GPSConfig *gps_config) {
  GPSHardware *gps_hw = new GPSHardware();

  if (!gps_hw->SetInterface(serial)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to set GPS interface!");
    delete gps_hw;
    return false;
  }

  if (!gps_hw->begin()) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to initialize GPS hardware!");
    delete gps_hw;
    return false;
  }

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
  WS_DEBUG_PRINT("[gps] Pushing NMEA sentence: ");
  WS_DEBUG_PRINTLN(_nmea_buff.sentences[_nmea_buff.head]);
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

    // Perform a keep-alive check by sending an antenna check command every 2
    // seconds
    if (millis() - drv->GetPrvKat() > 2000) {
      drv->GetAdaGps()->sendCommand(CMD_MTK_CHECK_ANTENNA);
      drv->SetPrvKat(millis());
    }

    // Did read period elapse?
    ulong cur_time = millis();
    if (cur_time - drv->GetPollPeriodPrv() < drv->GetPollPeriod())
      continue; // Not yet elapsed, skip this driver

    // Discard the GPS buffer before we attempt to do a fresh read
    size_t bytes_avail = drv->GetAdaGps()->available();
    if (bytes_avail > 0) {
      // TODO: Remove these two WS_DEBUG_PRINTs!
      WS_DEBUG_PRINT("[gps] Discarding GPS data: ");
      WS_DEBUG_PRINTLN(bytes_avail);
      // Read the available data from the GPS module
      // and discard it
      for (size_t i = 0; i < bytes_avail; i++) {
        drv->GetAdaGps()->read();
      }
    }

    // Unset the received flag
    if (drv->GetAdaGps()->newNMEAreceived()) {
      drv->GetAdaGps()->lastNMEA();
    }

    // Let's attempt to get a sentence from the GPS module
    // Convert the NMEA update rate to milliseconds
    // TODO: This should be stored as a member within the hardware class
    ulong update_rate = 1000 / drv->GetNmeaUpdateRate();

    // Read from the GPS module for update_rate milliseconds
    ulong start_time = millis();
    int read_calls = 0;

    WS_DEBUG_PRINT("[gps] Reading GPS data for ");
    WS_DEBUG_PRINT(update_rate);
    WS_DEBUG_PRINTLN(" milliseconds...");
    while (millis() - start_time < update_rate) {
      char c = drv->GetAdaGps()->read();
      read_calls++;

      // Check if we have a new NMEA sentence
      if (drv->GetAdaGps()->newNMEAreceived()) {
        // If we have a new sentence, push it to the buffer
        // TODO: Check result of this operation actually
        WS_DEBUG_PRINTLN("[gps] New data, pushing sentence to buffer.");
        // Push the last NMEA sentence to the buffer
        char *last_nmea = drv->GetAdaGps()->lastNMEA();
        NmeaBufPush(drv->GetAdaGps()->lastNMEA());
      }
    }
    WS_DEBUG_PRINTLN("[gps] Finished reading GPS data.");

    // We are done reading for this period

    // TODO: This is for debugging purposes only, remove later!
    WS_DEBUG_PRINT("[gps] Read ");
    WS_DEBUG_PRINT(read_calls);
    WS_DEBUG_PRINTLN(" times from GPS module.");
    // Pop off the buffer and parse
    char nmea_sentence[MAX_LEN_NMEA_SENTENCE];
    // Pop until we have no more sentences in the buffer
    while (NmeaBufPop(nmea_sentence) != -1) {
      // Parse the NMEA sentence
      WS_DEBUG_PRINT("[gps] Parsing NMEA sentence: ");
      WS_DEBUG_PRINTLN(nmea_sentence);
      if (!drv->GetAdaGps()->parse(nmea_sentence)) {
        WS_DEBUG_PRINT("[gps] ERROR: Failed to parse NMEA sentence: ");
        WS_DEBUG_PRINTLN(nmea_sentence);
        continue; // Skip this driver if parsing failed
      }
      Serial.print("Fix: ");
      Serial.print((int)drv->GetAdaGps()->fix);
      Serial.print(" quality: ");
      Serial.println((int)drv->GetAdaGps()->fixquality);
      if (drv->GetAdaGps()->fix) {
        Serial.print("Location: ");
        Serial.print(drv->GetAdaGps()->latitude, 4);
        Serial.print(drv->GetAdaGps()->lat);
        Serial.print(", ");
        Serial.print(drv->GetAdaGps()->longitude, 4);
        Serial.println(drv->GetAdaGps()->lon);
        Serial.print("Speed (knots): ");
        Serial.println(drv->GetAdaGps()->speed);
        Serial.print("Angle: ");
        Serial.println(drv->GetAdaGps()->angle);
        Serial.print("Altitude: ");
        Serial.println(drv->GetAdaGps()->altitude);
        Serial.print("Satellites: ");
        Serial.println((int)drv->GetAdaGps()->satellites);
        Serial.print("Antenna status: ");
        Serial.println((int)drv->GetAdaGps()->antenna);
      }
    }
    WS_DEBUG_PRINTLN("[gps] Finished processing NMEA sentences.");
    // TODO: Successfully parsed the NMEA sentence, update the model

    drv->SetPollPeriodPrv(cur_time);
  }
}