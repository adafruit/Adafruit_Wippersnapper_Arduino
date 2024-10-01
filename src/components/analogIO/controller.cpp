/*!
 * @file controller.cpp
 *
 * Controller for the analogio.proto API
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

AnalogIOController::AnalogIOController() {
    _analogio_hardware = new AnalogIOHardware();
    _analogio_model = new AnalogIOModel();
    _analogio_hardware->SetResolution(16); // Default to 16-bit resolution
    SetRefVoltage(0.0);                    // Default to 0.0V
}

AnalogIOController::~AnalogIOController() {}


void AnalogIOController::SetRefVoltage(float voltage) {
    _ref_voltage = voltage;
}

float AnalogIOController::GetRefVoltage(void) {
    return _ref_voltage;
}

bool AnalogIOController::Handle_AnalogIOAdd(pb_istream_t *stream) {
  // Attempt to decode the incoming message into an AnalogIOAdd object
  if (!_analogio_model->DecodeAnalogIOAdd(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode AnalogIOAdd message");
    return false;
  }

  // TODO TOMORROW: pin_name is a pb_callback_t, not an char, maybe an issue with
  // how options picks things up? First, check the newly compiled output bc may
  // have fixed
  int pin_name = atoi(_analogio_model->GetAnalogIOAddMsg()->pin_name + 1);
  // Create a new analogioPin object
  // TODO: Replicate this within the digitalio controller, much cleaner way to assign!
  analogioPin new_pin = {
    .name = pin_name,
    .period = long(_analogio_model->GetAnalogIOAddMsg()->period) * 1000,
    .prv_period = 0,
    .read_mode = _analogio_model->GetAnalogIOAddMsg()->read_mode
  };

  // TODO: Initialize the pin in the hardware layer
  _analogio_hardware->InitPin(pin_name);

  // Add the new pin to the vector
  _analogio_pins.push_back(new_pin);

  return true;
}