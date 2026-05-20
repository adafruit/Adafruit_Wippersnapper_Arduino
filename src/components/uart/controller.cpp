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
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
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
  // Delete drivers before ports — drivers hold a pointer to the
  // hardware's serial, so hardware must outlive the driver teardown.
  for (drvUartBase *drv : _uart_drivers) {
    delete drv;
  }
  _uart_drivers.clear();
  for (UARTHardware *hw : _ports) {
    delete hw;
  }
  _ports.clear();
  if (_uart_model != nullptr) {
    delete _uart_model;
    _uart_model = nullptr;
  }
}

/*!
    @brief  Routes messages using the uart.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool UARTController::Router(pb_istream_t *stream) {
  // Attempt to decode the UART B2D envelope
  ws_uart_B2D b2d = ws_uart_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_uart_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[uart] ERROR: Unable to decode UART B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_uart_B2D_add_tag:
    res = Handle_UartAdd(&b2d.payload.add);
    break;
  case ws_uart_B2D_remove_tag:
    res = Handle_UartRemove(&b2d.payload.remove);
    break;
  case ws_uart_B2D_write_tag:
    res = Handle_UartWrite(&b2d.payload.write);
    break;
  default:
    WS_DEBUG_PRINTLN("[uart] WARNING: Unsupported UART payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles a UartAdd message.
    @param  msg
            The UartAdd message.
    @return True if the message was handled successfully, False otherwise.
*/
bool UARTController::Handle_UartAdd(ws_uart_Add *msg) {
  if (!msg->has_cfg_serial && !msg->has_cfg_device) {
    Ws.error_controller->publishComponentError(msg->descriptor, "Missing serial/device configuration");
    return false;
  }

  // Have we already added this UART port?
  int port = msg->descriptor.uart_nbr;
  for (UARTHardware *hw : _ports) {
    if (hw->getPortNum() == port) {
      Ws.error_controller->publishComponentError(msg->descriptor, "Port already in use");
      return false;
    }
  }

  // Configure a new UART port using the provided serial configuration
  ws_uart_SerialConfig cfg_serial = msg->cfg_serial;
  UARTHardware *uart_hardware = new UARTHardware(cfg_serial, msg->descriptor.uart_nbr);
  if (!uart_hardware->ConfigureSerial()) {
    Ws.error_controller->publishComponentError(msg->descriptor, "Failed to configure UART hardware");
    delete uart_hardware;
    return false;
  }
  _ports.push_back(uart_hardware);

  // Create a new UartDevice "driver" on the hardware layer (UARTHardware)
  drvUartBase *uart_driver = nullptr;
  ws_sensor_Type *sensor_types = nullptr;
  size_t sensor_types_count = 0;
  float sensor_period = 0;

  switch (msg->descriptor.type) {
  case ws_uart_DeviceType_DT_GENERIC_INPUT:
    if (strcmp(msg->descriptor.id, "us100") != 0) {
      Ws.error_controller->publishComponentError(msg->descriptor, "Unsupported generic input device ID");
      delete uart_hardware;
      return false;
    }
    uart_driver = new drvUartUs100(uart_hardware->GetHardwareSerial(),
                                   msg->descriptor.id, msg->descriptor.uart_nbr);
    sensor_types = msg->cfg_device.config.generic_input.types;
    sensor_types_count = msg->cfg_device.config.generic_input.types_count;
    sensor_period = msg->cfg_device.config.generic_input.period;
    WS_DEBUG_PRINTLN("[uart] Added US-100 Ultrasonic Distance Sensor!");
    break;
  case ws_uart_DeviceType_DT_PM25AQI:
    // TODO: Support SoftwareSerial as well, currently only HardwareSerial
    uart_driver = new drvUartPm25(uart_hardware->GetHardwareSerial(),
                                  msg->descriptor.id, msg->descriptor.uart_nbr);
    sensor_types = msg->cfg_device.config.pm25aqi.types;
    sensor_types_count = msg->cfg_device.config.pm25aqi.types_count;
    sensor_period = msg->cfg_device.config.pm25aqi.period;
    WS_DEBUG_PRINTLN("[uart] Added PM2.5 AQI device!");
    break;
  case ws_uart_DeviceType_DT_GENERIC_OUTPUT:
  case ws_uart_DeviceType_DT_GPS:
  case ws_uart_DeviceType_DT_TM22XX:
    Ws.error_controller->publishComponentError(msg->descriptor, "Unsupported UART device type");
    delete uart_hardware;
    return false;
  default:
    Ws.error_controller->publishComponentError(msg->descriptor, "Unknown device type");
    delete uart_hardware;
    return false;
  }

  // Common driver configuration
  uart_driver->ConfigureDriver(msg->descriptor.type, msg->cfg_device);
  uart_driver->EnableSensorEvents(sensor_types, sensor_types_count);
  uart_driver->SetSensorPeriod(sensor_period);

  // Attempt to initialize the UART driver
  if (!uart_driver->begin()) {
    Ws.error_controller->publishComponentError(msg->descriptor, "Failed to initialize UART driver");
    delete uart_driver;
    return false;
  }
  _uart_drivers.push_back(uart_driver);

  // Print what we added on uart
  WS_DEBUG_PRINT("[uart] Added UART device: ");
  WS_DEBUG_PRINTVAR(msg->descriptor.id);
  WS_DEBUG_PRINT(" on port ");
  WS_DEBUG_PRINTVAR(msg->descriptor.uart_nbr);
  WS_DEBUG_PRINTLN();
  return true;
}

/*!
    @brief  Handles a UartRemove message.
    @param  msg
            The UartRemove message.
    @return True if the message was handled successfully, False otherwise.
*/
bool UARTController::Handle_UartRemove(ws_uart_Remove *msg) {

  // Find the corresponding hardware instance for the UART port
  uint32_t port_num = msg->descriptor.uart_nbr;
  for (std::vector<UARTHardware *>::iterator it = _ports.begin();
       it != _ports.end(); ++it) {
    if ((*it)->getPortNum() == port_num) {
      // Find the corresponding driver for the uart port
      for (std::vector<drvUartBase *>::iterator driver_it =
               _uart_drivers.begin();
           driver_it != _uart_drivers.end(); ++driver_it) {
        if ((*driver_it)->GetPortNum() == port_num &&
            (*driver_it)->GetDeviceType() == msg->descriptor.type &&
            strcmp((*driver_it)->GetName(), msg->descriptor.id) == 0) {
          // Driver found, remove it
          delete *driver_it;
          _uart_drivers.erase(driver_it);
          // Also remove and free the hardware port
          delete *it;
          _ports.erase(it);
          WS_DEBUG_PRINT("[uart] Removed UART device: ");
          WS_DEBUG_PRINTVAR(msg->descriptor.id);
          WS_DEBUG_PRINT(" on port ");
          WS_DEBUG_PRINTVAR(msg->descriptor.uart_nbr);
          WS_DEBUG_PRINTLN();
          return true;
        }
      }
      // Port found but no matching driver on it
      Ws.error_controller->publishComponentError(
          msg->descriptor, "No matching driver found on port");
      return false;
    }
  }

  Ws.error_controller->publishComponentError(msg->descriptor,
                                              "Port not found for removal");
  return false;
}

/*!
    @brief  Handles a UartWrite message.
    @param  msg
            The UartWrite message.
    @return True if the message was handled successfully, False otherwise.
*/
bool UARTController::Handle_UartWrite(ws_uart_Write *msg) {
  Ws.error_controller->publishComponentError(msg->descriptor, "UartWrite not implemented.");
  return false;
}

/*!
    @brief  Polls the UARTController for updates, processes any pending events
   from the UART drivers and sends data to Adafruit IO.
    @param  force
            If true, forces a read on all drivers regardless of period.
*/
void UARTController::update(bool force) {
  if (_uart_drivers.empty())
    return; // bail-out

  for (drvUartBase *drv : _uart_drivers) {
    // (force only) - Was driver previously read and sent?
    if (drv->GetDidReadSend() && force)
      continue;

    size_t num_sensors = drv->GetNumSensors();
    if (num_sensors == 0) {
      WS_DEBUG_PRINT("[uart] No sensors available for driver: ");
      WS_DEBUG_PRINTLNVAR(drv->GetName());
      continue; // No sensors to poll, skip this driver
    }

    // Did driver's read period elapse yet?
    ulong cur_time = millis();
    if (!force &&
        (cur_time - drv->GetSensorPeriodPrv() < drv->GetSensorPeriod()))
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
        drv->SetDidReadSend(false);
        continue; // skip this sensor if reading failed
      }
      // Fill the event with the sensor data
      _uart_model->AddUartInputEvent(event, drv->_sensors[i]);
    }

    // Encode the UART input event message
    if (_uart_model->EncodeUartInputEvent()) {
      // Handle the UartInputEvent message
      if (Ws._sdCardV2->isModeOffline()) {
        // TODO: This is UNIMPLEMENTED!
        // In offline mode, log to SD card
        /* if
        (!Ws._sdCardV2->LogUartInputEvent(_uart_model->GetUartInputEventMsg()))
        { WS_DEBUG_PRINTLN("[uart] ERROR: Unable to log the UartInputEvent to
        SD!"); statusLEDSolid(WS_LED_STATUS_FS_WRITE);
        drv->SetDidReadSend(false);
        } else {
        drv->SetDidReadSend(true);
        } */
        drv->SetDidReadSend(true); // For now, assume success in offline mode
      } else {
        // In online mode, publish to Adafruit IO
        if (!Ws.PublishD2b(ws_signal_BrokerToDevice_uart_tag,
                           _uart_model->GetUartInputEventMsg())) {
          WS_DEBUG_PRINTLN(
              "[uart] ERROR: Unable to publish UartInputEvent to IO!");
          drv->SetDidReadSend(false);
        } else {
          drv->SetDidReadSend(true);
        }
      }
    } else {
      WS_DEBUG_PRINTLN(
          "[uart] ERROR: Failed to encode UartInputEvent message!");
      drv->SetDidReadSend(false);
    }
    // Update the driver's previous period timestamp
    cur_time = millis();
    drv->SetSensorPeriodPrv(cur_time);
  }
}

/*!
    @brief  Checks if all UART drivers have been read and their values sent.
    @return True if all drivers have been read and sent, False otherwise.
*/
bool UARTController::UpdateComplete() {
  for (drvUartBase *drv : _uart_drivers) {
    if (!drv->GetDidReadSend()) {
      return false;
    }
  }
  return true;
}

/*!
    @brief  Resets all UART drivers' did_read_send flags to false.
*/
void UARTController::ResetFlags() {
  for (drvUartBase *drv : _uart_drivers) {
    drv->SetDidReadSend(false);
  }
}