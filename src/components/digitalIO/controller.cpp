/*!
 * @file model.cpp
 *
 * Controller for the digitalio.proto API
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

DigitalIOController::DigitalIOController() {}

DigitalIOController::~DigitalIOController() {}

bool DigitalIOController::AddDigitalPin(pb_istream_t *stream) {
  // Attempt to decode the DigitalIOAdd message and parse it into the model
  if (!WsV2.digital_io_model->DecodeDigitalIOAdd(stream))
    return false; // Failed to decode the DigitalIOAdd message
  WsV2.digital_io_model->ParseDigitalIOAdd();

  // Get the DigitalIOAdd message
  wippersnapper_digitalio_DigitalIOAdd *dio_add =
      WsV2.digital_io_model->GetDigitalIOAdd();

  // Configure the pin based on the direction
  if (dio_add->gpio_direction ==
      wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_OUTPUT) {
    DigitalOutputPin new_pin = DigitalOutputPin(
        dio_add->pin_name, dio_add->value); // TODO: This is pb_callback type
    _digital_output_pins.push_back(new_pin);
  } else if (
      dio_add->gpio_direction ==
          wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT ||
      dio_add->gpio_direction ==
          wippersnapper_digitalio_DigitalIODirection_DIGITAL_IO_DIRECTION_INPUT_PULL_UP) {
    // TODO
  } else {
    return false; // Invalid pin direction specified
  }

  // TODO: After we parse and add it into the controller, we should add a
  // clear() method to reset the message object

  return true;
}