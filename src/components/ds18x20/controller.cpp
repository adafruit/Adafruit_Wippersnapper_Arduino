/*!
 * @file src/components/ds18x20/controller.cpp
 *
 * Controller for the ds18x20.proto API
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
    @brief  DS18X20Controller constructor
*/
DS18X20Controller::DS18X20Controller() {
  _num_drivers = 0;
  _DS18X20_model = new DS18X20Model();
}

/*!
    @brief  DS18X20Controller destructor
*/
DS18X20Controller::~DS18X20Controller() {
  _num_drivers = 0;
  delete _DS18X20_model;
}

/*!
    @brief  Routes messages using the ds18x20.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool DS18X20Controller::Router(pb_istream_t *stream) {
  // Attempt to decode the DS18X20 B2D envelope
  ws_ds18x20_B2D b2d = ws_ds18x20_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_ds18x20_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[ds18x20] ERROR: Unable to decode DS18X20 B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_ds18x20_B2D_add_tag:
    res = Handle_Ds18x20Add(&b2d.payload.add);
    break;
  case ws_ds18x20_B2D_remove_tag:
    res = Handle_Ds18x20Remove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[ds18x20] WARNING: Unsupported DS18X20 payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles a Ds18x20Add message from the broker. Attempts to
            initialize a  OneWire bus on the requested pin, attempts to
            initialize a DSTherm driver on the OneWire bus, adds the
            OneWire bus to the controller, and publishes a Ds18x20Added
            message to the broker indicating the result of this function.
    @param  msg
            The Ds18x20Add message.
    @return True if the sensor was successfully initialized,
            added to the controller, and a response was succesfully
            published to the broker. False otherwise.
*/
bool DS18X20Controller::Handle_Ds18x20Add(ws_ds18x20_Add *msg) {
  WS_DEBUG_PRINTLN("[ds18x20] Handle_Ds18x20Add MESSAGE...");

  // If we receive no sensor types to configure, bail out
  if (msg->sensor_types_count == 0) {
    WS_DEBUG_PRINTLN("ERROR | DS18x20: No ds18x sensor types provided!");
    return false;
  }

  // Extract the OneWire pin from the message
  uint8_t pin_name = atoi(msg->onewire_pin + 1);

#ifdef ARDUINO_ARCH_SAMD
  auto new_dsx_driver = new DS18X20Hardware(pin_name, _num_drivers);
  std::unique_ptr<DS18X20Hardware> unique_driver(new_dsx_driver);
#else
  auto new_dsx_driver =
      std::make_unique<DS18X20Hardware>(pin_name, _num_drivers);
#endif
  // Attempt to get the sensor's ID on the OneWire bus to show it's been init'd
  bool is_initialized = new_dsx_driver->GetSensor();

  WS_DEBUG_PRINT("DS18x20 Sensor Initialization Status: ");
  WS_DEBUG_PRINTLN(is_initialized ? "SUCCESS" : "FAILURE");
  WS_DEBUG_PRINT("OneWire Pin: ");
  WS_DEBUG_PRINTLN(pin_name);
  WS_DEBUG_PRINT("Sensor # on Bus: ");
  WS_DEBUG_PRINTLN(_num_drivers);
  WS_DEBUG_PRINT("Sensor Type Count: ");
  WS_DEBUG_PRINTLN(msg->sensor_types_count);

  if (is_initialized) {
    WS_DEBUG_PRINTLN("Sensor found on OneWire bus and initialized");

    // Set the sensor's pin name (non-logical name)
    new_dsx_driver->setOneWirePinName(msg->onewire_pin);

    // Set the sensor's resolution
    new_dsx_driver->SetResolution(msg->sensor_resolution);

    // Set the sensor's period
    new_dsx_driver->SetPeriod(msg->period);

    // Configure the types of sensor reads to perform
    for (int i = 0; i < msg->sensor_types_count; i++) {
      if (msg->sensor_types[i] == ws_sensor_Type_T_OBJECT_TEMPERATURE) {
        new_dsx_driver->is_read_temp_c = true;
      } else if (msg->sensor_types[i] ==
                 ws_sensor_Type_T_OBJECT_TEMPERATURE_FAHRENHEIT) {
        new_dsx_driver->is_read_temp_f = true;
      } else {
        WS_DEBUG_PRINTLN(
            "ERROR | DS18x20: Unknown SensorType, failed to add sensor!");
        is_initialized = false;
      }
    }

    // If the sensor was successfully initialized, add it to the controller
    if (is_initialized == true) {
#ifdef ARDUINO_ARCH_SAMD
      _DS18X20_pins.push_back(std::move(unique_driver));
#else
      _DS18X20_pins.push_back(std::move(new_dsx_driver));
#endif
      _num_drivers++;
    }

    // Print out the details
    WS_DEBUG_PRINTLN("[ds18x] New Sensor Added!");
    WS_DEBUG_PRINT("\tPin: ");
    WS_DEBUG_PRINTLN(pin_name);
    WS_DEBUG_PRINT("\tResolution: ");
    WS_DEBUG_PRINTLN(msg->sensor_resolution);
    WS_DEBUG_PRINT("\tPeriod: ");
    WS_DEBUG_PRINTLN(msg->period);
    WS_DEBUG_PRINT("\tSI Units: ");
    for (int i = 0; i < msg->sensor_types_count; i++) {
      WS_DEBUG_PRINT(msg->sensor_types[i]);
      WS_DEBUG_PRINT(", ");
    }
    WS_DEBUG_PRINTLN("");
  } else {
    WS_DEBUG_PRINTLN("ERROR | DS18x20: Unable to get sensor ID!");
    is_initialized = false;
  }

  // If we're not in offline mode, publish a Ds18x20Added message back to the
  // broker
  if (!Ws._sdCardV2->isModeOffline()) {
    // Encode and publish a Ds18x20Added message back to the broker
    if (!_DS18X20_model->EncodeDS18x20Added(msg->onewire_pin, is_initialized)) {
      WS_DEBUG_PRINTLN(
          "ERROR | DS18x20: Unable to encode Ds18x20Added message!");
      return false;
    }

    if (!Ws.PublishD2b(ws_signal_DeviceToBroker_ds18x20_tag,
                         _DS18X20_model->GetDS18x20AddedMsg())) {
      WS_DEBUG_PRINTLN(
          "ERROR | DS18x20: Unable to publish Ds18x20Added message!");
      return false;
    }
  }

  return true;
}

/*!
    @brief  Handles a Ds18x20Remove message from the broker. Attempts to
            remove a DS18X20Hardware object from the controller and
            release the OneWire pin for other uses.
    @param  msg
            The Ds18x20Remove message.
    @return True if the sensor was successfully removed from the
            controller, False otherwise.
*/
bool DS18X20Controller::Handle_Ds18x20Remove(ws_ds18x20_Remove *msg) {
  WS_DEBUG_PRINT("[ds18x20] Handle_Ds18x20Remove MESSAGE...");

  // Get the pin name
  uint8_t pin_name = atoi(msg->onewire_pin + 1);

  // Find the driver/bus in the vector and remove it
  for (size_t i = 0; i < _DS18X20_pins.size(); ++i) {
    if (_DS18X20_pins[i]->GetOneWirePin() == pin_name) {
      _DS18X20_pins.erase(_DS18X20_pins.begin() + i);
      return true;
    }
  }
  WS_DEBUG_PRINTLN("Removed!");
  _num_drivers--;
  return true;
}

/*!
    @brief  Update/polling loop for the DS18X20 controller.
    @param  force
            If true, forces a read on all sensors regardless of period.
*/
void DS18X20Controller::update(bool force) {
#ifdef DEBUG_PROFILE
  unsigned long total_start_time = millis();
#endif

  // Bail out if there are no OneWire pins to poll
  if (_DS18X20_pins.empty())
    return;

  // Iterate through the vector
  for (uint8_t i = 0; i < _DS18X20_pins.size(); i++) {
#ifdef DEBUG_PROFILE
    unsigned long sensor_start_time = millis();
#endif

    // Create a reference to the DS18X20Hardware object
    DS18X20Hardware &temp_dsx_driver = *(_DS18X20_pins[i]);

    // (force only) - Was sensor previously read and sent?
    if (temp_dsx_driver.did_read_send && force)
      continue;

    // Check if the driver's timer has not expired yet
    if (!force && !temp_dsx_driver.IsTimerExpired()) {
      continue;
    }

// Attempt to read the temperature in Celsius
#ifdef DEBUG_PROFILE
    unsigned long temp_c_start_time = millis();
#endif

    if (!temp_dsx_driver.ReadTemperatureC()) {
      WS_DEBUG_PRINTLN(
          "ERROR | DS18x20: Unable to read temperature in Celsius");
      temp_dsx_driver.did_read_send = false;
      continue;
    }

#ifdef DEBUG_PROFILE
    unsigned long temp_c_end_time = millis();
    WS_DEBUG_PRINT("Read temperature Celsius time: ");
    WS_DEBUG_PRINTLN(temp_c_end_time - temp_c_start_time);
#endif

    // We got a temperature value from the hardware, let's create a new
    // sensor_event
    _DS18X20_model->InitDS18x20EventMsg(temp_dsx_driver.getOneWirePinName());

    // Are we reading the temperature in Celsius, Fahrenheit, or both?
    if (temp_dsx_driver.is_read_temp_c) {
      float temp_c = temp_dsx_driver.GetTemperatureC();
      _DS18X20_model->addSensorEvent(ws_sensor_Type_T_OBJECT_TEMPERATURE,
                                     temp_c);
    }
    if (temp_dsx_driver.is_read_temp_f) {
      float temp_f = temp_dsx_driver.GetTemperatureF();
      _DS18X20_model->addSensorEvent(
          ws_sensor_Type_T_OBJECT_TEMPERATURE_FAHRENHEIT, temp_f);
    }

    // Get the Ds18x20Event message
    ws_ds18x20_Event *event_msg = _DS18X20_model->GetDS18x20EventMsg();
    pb_size_t event_count = event_msg->sensor_events_count;

    if (!Ws._sdCardV2->isModeOffline()) {
      // Encode the Ds18x20Event message
      if (!_DS18X20_model->EncodeDs18x20Event()) {
        WS_DEBUG_PRINTLN(
            "ERROR | DS18x20: Failed to encode Ds18x20Event message");
        temp_dsx_driver.did_read_send = false;
        continue;
      }
      // Publish the Ds18x20Event message to the broker
      WS_DEBUG_PRINT("DS18x20: Publishing event to broker...");
      if (!Ws.PublishD2b(ws_signal_DeviceToBroker_ds18x20_tag,
                           _DS18X20_model->GetDS18x20EventMsg())) {
        WS_DEBUG_PRINTLN(
            "ERROR | DS18x20: Failed to publish Ds18x20Event message");
        temp_dsx_driver.did_read_send = false;
        continue;
      }
      WS_DEBUG_PRINTLN("Published!");
      temp_dsx_driver.did_read_send = true;
    } else {
      if (!Ws._sdCardV2->LogDS18xSensorEventToSD(
              _DS18X20_model->GetDS18x20EventMsg())) {
        WS_DEBUG_PRINTLN(
            "ERROR | DS18x20: Failed to log DS18x20 event to SD card");
        temp_dsx_driver.did_read_send = false;
        continue;
      }
      temp_dsx_driver.did_read_send = true;
    }

#ifdef DEBUG_PROFILE
    unsigned long sensor_end_time = millis();
    WS_DEBUG_PRINT("Total sensor processing time: ");
    WS_DEBUG_PRINTLN(sensor_end_time - sensor_start_time);
#endif
  }

#ifdef DEBUG_PROFILE
  unsigned long total_end_time = millis();
  if (total_end_time - total_start_time != 0) {
    WS_DEBUG_PRINT("Total update() execution time: ");
    WS_DEBUG_PRINTLN(total_end_time - total_start_time);
  }
#endif
}

/*!
    @brief  Checks if all DS18X20 sensors have been read and their values sent.
    @return True if all sensors have been read and sent, False otherwise.
*/
bool DS18X20Controller::UpdateComplete() {
  for (size_t i = 0; i < _DS18X20_pins.size(); i++) {
    if (!_DS18X20_pins[i]->did_read_send) {
      return false;
    }
  }
  return true;
}
