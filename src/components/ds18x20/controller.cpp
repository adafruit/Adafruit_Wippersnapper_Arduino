/*!
 * @file controller.cpp
 *
 * Controller for the ds18x20.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

DS18X20Controller::DS18X20Controller() { _DS18X20_model = new DS18X20Model(); }

DS18X20Controller::~DS18X20Controller() { delete _DS18X20_model; }

bool DS18X20Controller::Handle_Ds18x20Add(pb_istream_t *stream) {
  // Attempt to decode the incoming message into a Ds18x20Add message
  if (!_DS18X20_model->DecodeDS18x20Add(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode Ds18x20Add message");
    return false;
  }
  // Extract the OneWire pin from the message
  uint8_t pin_name = atoi(_DS18X20_model->GetDS18x20AddMsg()->onewire_pin + 1);

  // Initialize the DS18X20Hardware object
  DS18X20Hardware new_dsx_driver(pin_name);
  // Attempt to get the sensor's ID on the OneWire bus to show it's been init'd
  bool is_initialized = new_dsx_driver.GetSensor();
  if (!is_initialized) {
    WS_DEBUG_PRINTLN("Sensor found on OneWire bus and initialized");
    // Set the sensor's resolution
    new_dsx_driver.setResolution(
        _DS18X20_model->GetDS18x20AddMsg()->sensor_resolution);

    // Add the DS18X20Hardware object to the vector of hardware objects
    _DS18X20_pins.push_back(new_dsx_driver);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to get sensor ID, sensor not initialized");
  }

  // Encode and publish a Ds18x20Added message back to the broker
  if (!_DS18X20_model->EncodeDS18x20Added(
          _DS18X20_model->GetDS18x20AddMsg()->onewire_pin, is_initialized)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode Ds18x20Added message");
    return false;
  }

  // Publish the AnalogIO message to the broker
  if (!WsV2.PublishSignal(wippersnapper_signal_DeviceToBroker_ds18x20_added_tag,
                          _DS18X20_model->GetDS18x20AddedMsg())) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish Ds18x20Added message");
    return false;
  }

  return true;
}

void DS18X20Controller::update() {
  // Bail out if there are no OneWire pins to poll
  if (_DS18X20_pins.size() == 0)
    return;

  // Iterate through the vector
  for (uint8_t i = 0; i < _DS18X20_pins.size(); i++) {
    // Create a temporary DS18X20Hardware driver
    DS18X20Hardware *temp_dsx_driver = &_DS18X20_pins[i];
    // Check if the driver's timer has expired
    if (temp_dsx_driver->IsTimerExpired())
      return;
    // TODO: the update() method should check sensor type(s)
    // before polling ReadTemperatureX methods!
  }
}