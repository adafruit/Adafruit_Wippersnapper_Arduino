/*!
 * @file src/components/uart/controller.cpp
 *
 * Controller for WipperSnapper's UART component, bridges between the UART.proto
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
    @brief  Constructs a new UARTController.
*/
UARTController::UARTController() { _uart_model = new UARTModel(); }

/*!
    @brief  Destructs the UARTController.
*/
UARTController::~UARTController() {
  if (_uart_model != nullptr) {
    delete _uart_model; // cleanup model
    _uart_model = nullptr;
  }
}

/*!
    @brief  Handles a UartAdd message.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was handled successfully, False otherwise.
*/
bool UARTController::Handle_UartAdd(pb_istream_t *stream) {
  // Attempt to decode the UartAdd message
  WS_DEBUG_PRINTLN("[uart] Decoding UartAdd message...");
  if (!_uart_model->DecodeUartAdd(stream)) {
    WS_DEBUG_PRINTLN("[uart] ERROR: Failed to decode UartAdd message!");
    return false;
  }
  WS_DEBUG_PRINTLN("[uart] UartAdd message decoded successfully!");
  // Get ref. to the UartAdd message within the model
  ws_uart_Add *add_msg = _uart_model->GetUartAddMsg();
  // TODO: fix the id field, currently it is a callback and should be a string.
  if (!add_msg->has_cfg_serial && !add_msg->has_cfg_device) {
    WS_DEBUG_PRINTLN(
        "[uart] ERROR: No configuration provided for UART device!");
    return false;
  }

  // Configure a UART hardware instance using the provided serial configuration
  // TODO: Have we already configured this UART hardware instance?!
  WS_DEBUG_PRINTLN("[uart] Configuring UART hardware instance...");
  ws_uart_SerialConfig cfg_serial = add_msg->cfg_serial;
  UARTHardware *uart_hardware = new UARTHardware(cfg_serial);
  if (!uart_hardware->ConfigureSerial()) {
    WS_DEBUG_PRINTLN("[uart] ERROR: Failed to configure UART hardware!");
    delete uart_hardware; // cleanup
    return false;
  }
  // Add the newly configured hardware instance to the controller's vector of
  // UART ports
  _uart_ports.push_back(uart_hardware);
  WS_DEBUG_PRINTLN("[uart] UART hardware instance configured successfully!");

  // Create a new UartDevice "driver" on the hardware layer (UARTHardware)
  // TODO: Have we already added this UART device?!
  drvUartBase *uart_driver = nullptr;
  GPSController *drv_uart_gps = nullptr;
  bool is_gps_drv = false;
  ws_uart_DeviceConfig cfg_device = add_msg->cfg_device;
  switch (cfg_device.device_type) {
  case ws_uart_DeviceType_DT_UNSPECIFIED:
    WS_DEBUG_PRINTLN("[uart] ERROR: Unspecified device type!");
    return false;
  case ws_uart_DeviceType_DT_GENERIC_INPUT:
    // check if device_type is "us100"
    if (strcmp(cfg_device.device_id, "us100") == 0) {
      WS_DEBUG_PRINTLN("[uart] Adding US-100 device..");
      // Create a new US-100 driver instance
      WS_DEBUG_PRINT("[uart] Adding US-100 Driver...");
      uart_driver = new drvUartUs100(uart_hardware->GetHardwareSerial(),
                                     cfg_device.device_id, cfg_serial.uart_nbr);
      uart_driver->ConfigureDriver(cfg_device);
      uart_driver->EnableSensorEvents(
          cfg_device.config.generic_input.sensor_types,
          cfg_device.config.generic_input.sensor_types_count);
      uart_driver->SetSensorPeriod(cfg_device.config.generic_input.period);
      WS_DEBUG_PRINT("added!");
    } else {
      WS_DEBUG_PRINTLN(
          "[uart] Specified generic device type is not implemented!");
      delete uart_hardware; // cleanup
      return false;
    }
    break;
  case ws_uart_DeviceType_DT_GENERIC_OUTPUT:
    WS_DEBUG_PRINTLN("[uart] Generic Output device type not implemented!");
    delete uart_hardware; // cleanup
    return false;
  case ws_uart_DeviceType_DT_GPS:
    WS_DEBUG_PRINTLN("[uart] Adding GPS device..");
    if (!WsV2._gps_controller->AddGPS(uart_hardware->GetHardwareSerial(),
                                      &cfg_device.config.gps)) {
      WS_DEBUG_PRINTLN("[uart] ERROR: Failed to initialize GPS device!");
      delete uart_hardware; // cleanup
      return false;
    }
    WS_DEBUG_PRINTLN("[uart] Added GPS driver!");
    is_gps_drv = true; // mark as GPS driver
    break;
  case ws_uart_DeviceType_DT_PM25AQI:
    WS_DEBUG_PRINTLN("[uart] Adding PM2.5 AQI device..");
    // Create a new PM2.5 AQI driver instance
    // TODO: Support SoftwareSerial as well, currently only HardwareSerial
    uart_driver = new drvUartPm25(uart_hardware->GetHardwareSerial(),
                                  cfg_device.device_id, cfg_serial.uart_nbr);
    uart_driver->ConfigureDriver(cfg_device);
    uart_driver->EnableSensorEvents(
        cfg_device.config.pm25aqi.sensor_types,
        cfg_device.config.pm25aqi.sensor_types_count);
    uart_driver->SetSensorPeriod(cfg_device.config.pm25aqi.period);
    WS_DEBUG_PRINT("added!");
    break;
  case ws_uart_DeviceType_DT_TM22XX:
    WS_DEBUG_PRINTLN("[uart] TM22XX device type not implemented!");
    delete uart_hardware; // cleanup
    return false;
  default:
    WS_DEBUG_PRINTLN("[uart] ERROR: Unknown device type!");
    delete uart_hardware; // cleanup
    return false;
  }

  // Attempt to initialize the UART driver
  bool did_begin = false;
  WS_DEBUG_PRINTLN("[uart] Initializing UART driver...");
  if (!is_gps_drv) {
    WS_DEBUG_PRINTLN("[uart] STD UART drv...");
    did_begin = uart_driver->begin();
    if (did_begin) {
      WS_DEBUG_PRINTLN("[uart] UART driver initialized successfully!");
      _uart_drivers.push_back(uart_driver);
    } else {
      WS_DEBUG_PRINTLN("[uart] ERROR: Failed to initialize UART driver!");
      delete uart_driver; // cleanup
      return false;
    }
  }

  // Are we in offline mode?
  if (WsV2._sdCardV2->isModeOffline())
    return true; // Don't publish to IO in offline mode

  // Encode and publish out to Adafruit IO
  WS_DEBUG_PRINTLN("[uart] Encoding UartAdded message...");
  if (!_uart_model->EncodeUartAdded(uart_hardware->GetBusNumber(),
                                    cfg_device.device_type,
                                    cfg_device.device_id, did_begin)) {
    WS_DEBUG_PRINTLN("[uart] ERROR: Failed to encode UartAdded message!");
    return false;
  }
  WS_DEBUG_PRINTLN("[uart] UartAdded message encoded successfully!");

  WS_DEBUG_PRINTLN("[uart] Publishing UartAdded message to IO...");
  // TODO: Unsure why this is causing a crash on GPS, figure out later
  // Currently commented out to prevent crashes
  /*   if
    (!WsV2.PublishD2b(wippersnapper_signal_DeviceToBroker_uart_added_tag,
                            _uart_model->GetUartAddedMsg())) {
      WS_DEBUG_PRINTLN("[i2c] ERROR: Unable to publish UartAdded message to
    IO!"); return false;
    } */
  WS_DEBUG_PRINTLN("[uart] UartAdded message published successfully!");

  return true;
}

/*!
    @brief  Handles a UartRemove message.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was handled successfully, False otherwise.
*/
bool UARTController::Handle_UartRemove(pb_istream_t *stream) {
  if (!_uart_model->DecodeUartDeviceRemove(stream)) {
    WS_DEBUG_PRINTLN("[uart] ERROR: Failed to decode UartRemove message!");
    return false;
  }
  // Get the UartRemove message from the model
  ws_uart_Remove *remove_msg = _uart_model->GetUartRemoveMsg();

  // Find the corresponding hardware instance for the UART port
  uint32_t port_num = remove_msg->descriptor.uart_nbr;
  for (auto it = _uart_ports.begin(); it != _uart_ports.end(); ++it) {
    if ((*it)->GetBusNumber() == port_num) {
      // Find the corresponding driver for the uart port
      for (auto driver_it = _uart_drivers.begin();
           driver_it != _uart_drivers.end(); ++driver_it) {
        if ((*driver_it)->GetPortNum() == port_num &&
            (*driver_it)->GetDeviceType() == remove_msg->descriptor.type &&
            strcmp((*driver_it)->GetName(),
                   (const char *)remove_msg->descriptor.device_id.arg) == 0) {
          // Driver found, remove it
          WS_DEBUG_PRINT("[uart] Removing UART driver: " +
                         String((*driver_it)->GetName()) + "...");
          delete *driver_it;
          _uart_drivers.erase(driver_it);
          WS_DEBUG_PRINTLN("Removed!");
          return true;
        }
      }
    }
  }

  return false;
}

/*!
    @brief  Handles a UartWrite message.
    @param  stream
            Pointer to a pb_istream_t object.
    @return True if the message was handled successfully, False otherwise.
*/
bool UARTController::Handle_UartWrite(pb_istream_t *stream) {
  // TODO: Needs implementation
  // TO ADDRESS:
  // 1) Hardware is the uart_nbr
  // 2) type is the driver type
  // 3) device_id is the unique identifier for the UART device, stored by the
  // driver
  return false;
}

/*!
    @brief  Polls the UARTController for updates, processes any pending events
   from the UART drivers and sends data to Adafruit IO.
*/
void UARTController::update() {
  if (_uart_drivers.empty())
    return; // bail-out

  for (drvUartBase *drv : _uart_drivers) {

    size_t num_sensors = drv->GetNumSensors();
    if (num_sensors == 0) {
      WS_DEBUG_PRINTLN("[uart] No sensors available for driver: " +
                       String(drv->GetName()));
      continue; // No sensors to poll, skip this driver
    }

    // Did driver's read period elapse yet?
    ulong cur_time = millis();
    if (cur_time - drv->GetSensorPeriodPrv() < drv->GetSensorPeriod())
      continue;

    // Read the events from the drivers
    _uart_model->ClearUartInputEventMsg();
    _uart_model->ConfigureUartInputEventMsg(
        drv->GetPortNum(), drv->GetDeviceType(), drv->GetName());
    for (size_t i = 0; i < num_sensors; i++) {
      // Attempt to read from the driver
      sensors_event_t event = {0};
      if (!drv->GetSensorEvent(drv->_sensors[i], &event)) {
        WS_DEBUG_PRINTLN("[uart] ERROR: Failed to read sensor!");
        continue; // skip this sensor if reading failed
      }
      // Fill the event with the sensor data
      _uart_model->AddUartInputEvent(event, drv->_sensors[i]);
    }

    // Encode the UART input event message
    if (_uart_model->EncodeUartInputEvent()) {
      // Handle the UartInputEvent message
      if (WsV2._sdCardV2->isModeOffline()) {
        // TODO: This is UNIMPLEMENTED!
        // In offline mode, log to SD card
        /* if
        (!WsV2._sdCardV2->LogUartInputEvent(_uart_model->GetUartInputEventMsg()))
        { WS_DEBUG_PRINTLN("[uart] ERROR: Unable to log the UartInputEvent to
        SD!"); statusLEDSolid(WS_LED_STATUS_FS_WRITE);
        } */
      } else {
        // In online mode, publish to Adafruit IO
        if (!WsV2.PublishD2b(ws_signal_BrokerToDevice_uart_tag,
                             _uart_model->GetUartInputEventMsg())) {
          WS_DEBUG_PRINTLN(
              "[uart] ERROR: Unable to publish UartInputEvent to IO!");
        }
      }
    } else {
      WS_DEBUG_PRINTLN(
          "[uart] ERROR: Failed to encode UartInputEvent message!");
    }
    // Update the driver's previous period timestamp
    cur_time = millis();
    drv->SetSensorPeriodPrv(cur_time);
  }
}