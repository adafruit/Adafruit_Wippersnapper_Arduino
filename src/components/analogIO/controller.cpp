/*!
 * @file src/components/analogIO/controller.cpp
 *
 * Controller for the analogio.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024-2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

/*!
    @brief  AnalogIO controller constructor
*/
AnalogIOController::AnalogIOController() {
  _analogio_hardware = new AnalogIOHardware();
  _analogio_model = new AnalogIOModel();
}

/*!
    @brief  AnalogIO controller destructor
*/
AnalogIOController::~AnalogIOController() {
  delete _analogio_hardware;
  delete _analogio_model;
}

/*!
    @brief  Set the reference voltage for the analog pins
    @param  voltage
            The reference voltage.
*/
void AnalogIOController::SetRefVoltage(float voltage) {
  // To set the reference voltage, we require a call into the hardware
  _analogio_hardware->SetReferenceVoltage(voltage);
}

/*!
    @brief  Allocate memory for the total number of analog pins.
    @param  total_pins
            The hardware's total number of analog pins.
*/
void AnalogIOController::SetTotalAnalogPins(uint8_t total_pins) {
  _analogio_pins.reserve(total_pins);
}

/*!
    @brief  Routes messages using the analogio.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool AnalogIOController::Router(pb_istream_t *stream) {
  // Attempt to decode the AnalogIO B2D envelope
  ws_analogio_B2D b2d = ws_analogio_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_analogio_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN(
        "[analogio] ERROR: Unable to decode AnalogIO B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_analogio_B2D_add_tag:
    res = Handle_AnalogIOAdd(&b2d.payload.add);
    break;
  case ws_analogio_B2D_remove_tag:
    res = Handle_AnalogIORemove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[analogio] WARNING: Unsupported AnalogIO payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles an AnalogIOAdd message from the broker and adds a
            new analog pin to the controller.
    @param  msg
            The AnalogIOAdd message.
    @return True if the pin was successfully added, False otherwise.
*/
bool AnalogIOController::Handle_AnalogIOAdd(ws_analogio_Add *msg) {
  WS_DEBUG_PRINTLN("[analogio] Handle_AnalogIOAdd MESSAGE...");
  // Get the pin name
  uint8_t pin_name = atoi(msg->pin_name + 1);

  // Create a new analogioPin object
  analogioPin new_pin = {
      .name = pin_name,
      .period = (ulong)(msg->period * 1000.0f),
      .prv_period =
          0, // Initialize previous period to 0 to force immediate read
      .read_mode = msg->read_mode};

  // Initialize the pin and add it to the vector
  _analogio_hardware->InitPin(pin_name);
  _analogio_pins.push_back(new_pin);

  // Print out the pin's details
  WS_DEBUG_PRINTLN("[analogio] Added new pin:");
  WS_DEBUG_PRINT("Pin Name: ");
  WS_DEBUG_PRINTLN(new_pin.name);
  WS_DEBUG_PRINT("Period: ");
  WS_DEBUG_PRINTLN(new_pin.period);
  WS_DEBUG_PRINT("Read Mode: ");
  WS_DEBUG_PRINTLN(new_pin.read_mode);

  return true;
}

/*!
    @brief  Handles an AnalogIORemove message from the broker and removes
            the requested analog pin from the controller.
    @param  msg
            The AnalogIORemove message.
    @return True if the pin was successfully removed, False otherwise.
*/
bool AnalogIOController::Handle_AnalogIORemove(ws_analogio_Remove *msg) {
  // Get the pin name
  uint8_t pin_name = atoi(msg->pin_name + 1);

  bool removed = false;
  for (size_t i = 0; i < _analogio_pins.size(); i++) {
    if (_analogio_pins[i].name == pin_name) {
      _analogio_hardware->DeinitPin(pin_name);
      _analogio_pins.erase(_analogio_pins.begin() + i);
      removed = true;
      break;
    }
  }

  if (!removed) {
    WS_DEBUG_PRINTLN("[analogio] ERROR: Unable to find requested pin!");
    return false;
  }

  WS_DEBUG_PRINT("[analogio] Removed pin: ");
  WS_DEBUG_PRINTLN(msg->pin_name);
  return true;
}

/*!
    @brief  Checks if a pin's periodic timer has expired.
    @param  pin
            The requested pin to check.
    @param  cur_time
            The current time (called from millis()).
    @return True if the pin's period has expired, False otherwise.
*/
bool AnalogIOController::IsPinTimerExpired(analogioPin *pin, ulong cur_time) {
  return cur_time - pin->prv_period > pin->period;
}

/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker.
    @param  pin
            The pin to encode and publish.
    @param  value
            The pin's value.
    @param  read_type
            The type of read to perform on the pin.
    @return True if the message was successfully encoded and published.
*/
bool AnalogIOController::EncodePublishPinEvent(uint8_t pin, float value,
                                               ws_sensor_Type read_type) {
  char c_pin_name[12];
  sprintf(c_pin_name, "A%d", pin);

  if (read_type == ws_sensor_Type_T_RAW) {
    if (!_analogio_model->EncodeAnalogIOEventRaw(c_pin_name, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIO raw adc message!");
      return false;
    }
  } else if (read_type == ws_sensor_Type_T_VOLTAGE) {
    if (!_analogio_model->EncodeAnalogIOEventVoltage(c_pin_name, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIO voltage message!");
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid read type for AnalogIOEvent message!");
    return false;
  }

  // Publish the AnalogIO message to the broker
  WS_DEBUG_PRINTLN("Publishing AnalogIOEvent message to broker...");
  if (!WsV2.PublishD2b(ws_signal_DeviceToBroker_analogio_tag,
                       _analogio_model->GetAnalogIOEvent())) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish analogio voltage event message, "
                     "moving onto the next pin!");
    return false;
  }
  WS_DEBUG_PRINTLN("Published AnalogIOEvent message to broker!")

  return true;
}

/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker or SD
            card, depending on the device's mode.
    @param  pin
            The requested pin.
    @param  value
            The pin's value.
    @return True if the message was successfully encoded and published,
            otherwise False.
*/
bool AnalogIOController::EncodePublishPinValue(uint8_t pin, uint16_t value) {
  if (!WsV2._sdCardV2->isModeOffline()) {
    return EncodePublishPinEvent(pin, (float)value, ws_sensor_Type_T_RAW);
  } else {
    return WsV2._sdCardV2->LogGPIOSensorEventToSD(pin, value,
                                                  ws_sensor_Type_T_RAW);
  }
}

/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker or SD
            card, depending on the device's mode.
    @param  pin
            The requested pin.
    @param  value
            The pin's value, as a voltage.
    @return True if the message was successfully encoded and published,
            otherwise False.
*/
bool AnalogIOController::EncodePublishPinVoltage(uint8_t pin, float value) {
  if (!WsV2._sdCardV2->isModeOffline()) {
    return EncodePublishPinEvent(pin, value, ws_sensor_Type_T_VOLTAGE);
  }
  return WsV2._sdCardV2->LogGPIOSensorEventToSD(pin, value,
                                                ws_sensor_Type_T_VOLTAGE);
}

/*!
    @brief  Update/polling loop for the AnalogIO controller.
*/
void AnalogIOController::update() {
  // Bail-out if the vector is empty
  if (_analogio_pins.empty()) {
    return;
  }

  // Process analog input pins
  size_t num_pins = _analogio_pins.size();
  for (size_t i = 0; i < num_pins; i++) {
    // Create a pin object for this iteration
    analogioPin &pin = _analogio_pins[i];
    // Go to the next pin if the period hasn't expired yet
    ulong cur_time = millis();
    if (!IsPinTimerExpired(&pin, cur_time))
      continue;

    // Pins timer has expired, lets read the pin
    if (pin.read_mode == ws_sensor_Type_T_RAW) {
      // Read the pin's raw value
      uint16_t value = _analogio_hardware->GetPinValue(pin.name);
      // Encode and publish it to the broker
      if (!EncodePublishPinValue(pin.name, value)) {
        WS_DEBUG_PRINTLN("[analogio] ERROR: Unable to record pin value!");
        continue;
      }
    } else if (pin.read_mode == ws_sensor_Type_T_VOLTAGE) {
      // Convert the raw value into voltage
      float pin_value = _analogio_hardware->GetPinVoltage(pin.name);
      // Encode and publish the voltage value to the broker
      if (!EncodePublishPinVoltage(pin.name, pin_value)) {
        WS_DEBUG_PRINTLN("[analogio] ERROR: Unable to record pin voltage!");
        continue;
      }
    } else {
      WS_DEBUG_PRINTLN("[analogio] ERROR: Invalid read mode for analog pin!");
      continue;
    }
    pin.prv_period = cur_time; // Reset the pin's period timer
  }
}