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
GPSController::GPSController() { _gps_model = new GPSModel(); }

/*!
 * @brief Destructor for GPSController.
 */
GPSController::~GPSController() {
  // Clean up model
  if (_gps_model) {
    delete _gps_model;
    _gps_model = nullptr;
  }
  // Clean up UART ports owned by GPS
  for (UARTHardware *port : _uart_ports) {
    delete port;
  }
  _uart_ports.clear();
}

/*!
 * @brief Routes messages using the gps.proto API to the
 *        appropriate controller functions.
 * @param stream The nanopb input stream.
 * @return True if the message was successfully routed, False otherwise.
 */
bool GPSController::Router(pb_istream_t *stream) {
  ws_gps_B2D b2d = ws_gps_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_gps_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Unable to decode GPS B2D envelope");
    return false;
  }

  bool res = false;
  switch (b2d.which_payload) {
  case ws_gps_B2D_device_add_replace_tag:
    res = Handle_GpsDeviceAddOrReplace(&b2d.payload.device_add_replace);
    break;
  case ws_gps_B2D_device_remove_tag:
    res = Handle_GpsDeviceRemove(&b2d.payload.device_remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[gps] WARNING: Unsupported GPS payload");
    res = false;
    break;
  }
  return res;
}

/*!
 * @brief Handles a GPS DeviceAddOrReplace message.
 * @param msg The DeviceAddOrReplace message.
 * @return True if the GPS device was added successfully, False otherwise.
 */
bool GPSController::Handle_GpsDeviceAddOrReplace(
    ws_gps_DeviceAddOrReplace *msg) {
  if (!msg->has_config) {
    WS_DEBUG_PRINTLN("[gps] ERROR: No GPS config provided!");
    return false;
  }

  bool added = false;

  if (msg->has_add_uart) {
    // UART transport path
    WS_DEBUG_PRINTLN("[gps] Configuring GPS via UART transport...");
    UARTHardware *uart_hw = new UARTHardware(msg->add_uart.cfg_serial);
    if (!uart_hw->ConfigureSerial()) {
      WS_DEBUG_PRINTLN("[gps] ERROR: Failed to configure UART hardware!");
      delete uart_hw;
      return false;
    }
    _uart_ports.push_back(uart_hw);

    added = AddGPS(uart_hw->GetHardwareSerial(), &msg->config);
  } else if (msg->has_add_i2c) {
    // I2C transport path
    WS_DEBUG_PRINTLN("[gps] Configuring GPS via I2C transport...");
    ws_i2c_DeviceDescriptor desc = msg->add_i2c.device_description;
    TwoWire *wire = Ws._i2c_controller->GetI2cBus(desc.pin_scl, desc.pin_sda);
    if (wire == nullptr) {
      WS_DEBUG_PRINTLN("[gps] ERROR: I2C bus not found for GPS device!");
      return false;
    }
    added = AddGPS(wire, desc.device_address, &msg->config);
  } else {
    WS_DEBUG_PRINTLN("[gps] ERROR: No transport (UART or I2C) specified!");
    return false;
  }

  if (!added) {
    WS_DEBUG_PRINTLN("[gps] ERROR: Failed to add GPS device!");
  }
  return added;
}

/*!
 * @brief Handles a GPS DeviceRemove message.
 * @param msg The DeviceRemove message.
 * @return True if the GPS device was removed successfully, False otherwise.
 */
bool GPSController::Handle_GpsDeviceRemove(ws_gps_DeviceRemove *msg) {
  // TODO: Implement GPS device removal
  (void)msg;
  WS_DEBUG_PRINTLN("[gps] GPS device removal not yet implemented");
  return false;
}

/*!
 * @brief Adds an I2C GPS hardware instance to the controller.
 * @param wire Pointer to the TwoWire instance for I2C communication.
 * @param i2c_addr I2C address of the GPS device.
 * @param gps_config Pointer to the GPS configuration message.
 * @return True if the GPS was added successfully, false otherwise.
 */
bool GPSController::AddGPS(TwoWire *wire, uint32_t i2c_addr,
                           ws_gps_Config *gps_config) {
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

  gps_hw->SetDidReadSend(false);
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
bool GPSController::AddGPS(HardwareSerial *serial, ws_gps_Config *gps_config) {
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

  gps_hw->SetDidReadSend(false);
  _gps_drivers.push_back(gps_hw);
  WS_DEBUG_PRINTLN("[gps] GPS hardware added successfully!");
  return true;
}

/*!
 * @brief Updates the GPSController, polling the GPS hardware for data.
 * This function checks if the read period has elapsed and processes the GPS
 * data accordingly.
 * @param force If true, forces a read on all GPS modules regardless of
 * period.
 */
void GPSController::update(bool force) {
  if (_gps_drivers.empty())
    return; // bail-out!

  for (GPSHardware *drv : _gps_drivers) {
    // (force only) - Was driver previously read and sent?
    if (drv->GetDidReadSend())
      continue;

    // TODO: Commented out due to parsing failures, stability issue (failed to
    // parse NMEA acks for this) Perform a keep-alive check by sending an
    // antenna check command every 2 seconds
    /*     if (millis() - drv->GetPrvKat() > 2000) {
          drv->GetAdaGps()->sendCommand(CMD_MTK_CHECK_ANTENNA);
          drv->SetPrvKat(millis());
        } */

    // Did read period elapse?
    ulong cur_time = millis();
    if (!force && (cur_time - drv->GetPollPeriodPrv() < drv->GetPollPeriod()))
      continue; // Not yet elapsed, skip this driver

    // Discard the GPS buffer before we attempt to do a fresh read
    drv->ReadDiscardBuffer();

    // Poll the GPS hardware for update_rate ms
    // and store NMEA sentences in a ring buffer
    drv->PollStoreSentences();

    // Parse each NMEA sentence in the buffer
    _gps_model->CreateGPSEvent();
    char nmea_sentence[MAX_LEN_NMEA_SENTENCE];
    bool has_gps_event = false;
    while (drv->NmeaBufPop(nmea_sentence) != -1) {
      // Let the driver parse the NMEA sentence
      WS_DEBUG_PRINT("[gps] Parsing NMEA sentence: ");
      WS_DEBUG_PRINTLNVAR(nmea_sentence);
      if (!drv->ParseNMEASentence(nmea_sentence)) {
        continue; // Skip this sentence if parsing failed
      } else {
        has_gps_event = true; // We have a valid NMEA sentence
      }

      // Using the Model, process the NMEA sentence into a GPSEvent
      WS_DEBUG_PRINTLN("[gps] Processing NMEA sentence...");
      _gps_model->ProcessNMEASentence(nmea_sentence, drv);
    }

    // Only try to encode/publish if we had at least one valid GPS event
    if (has_gps_event) {
      // Encode and publish to IO
      WS_DEBUG_PRINT("[gps] Encoding and publishing GPSEvent to IO...");
      bool did_encode = _gps_model->EncodeGPSEvent();
      if (!did_encode) {
        WS_DEBUG_PRINTLN("[gps] ERROR: Failed to encode GPSEvent!");
        drv->SetDidReadSend(false);
      } else {
        // Publish the GPSEvent to IO
        if (!Ws.PublishD2b(ws_signal_DeviceToBroker_gps_tag,
                           _gps_model->GetGPSD2B())) {
          WS_DEBUG_PRINTLN("[gps] ERROR: Failed to publish GPSEvent!");
          drv->SetDidReadSend(false);
        } else {
          WS_DEBUG_PRINTLN("...ok!");
          drv->SetDidReadSend(true);
        }
      }
    } else {
      // No valid GPS events were parsed
      drv->SetDidReadSend(false);
    }

    // Update the period timestamp
    drv->SetPollPeriodPrv(cur_time);
  }
}

/*!
 * @brief Checks if all GPS drivers have been read and their values sent.
 * @return True if all drivers have been read and sent, False otherwise.
 */
bool GPSController::UpdateComplete() {
  for (GPSHardware *drv : _gps_drivers) {
    if (!drv->GetDidReadSend()) {
      return false;
    }
  }
  return true;
}

/*!
 * @brief Resets all GPS drivers' did_read_send flags to false.
 */
void GPSController::ResetFlags() {
  for (GPSHardware *drv : _gps_drivers) {
    drv->SetDidReadSend(false);
  }
}