/*!
 * @file src/components/digitalIO/controller.cpp
 *
 * Controller for the digitalio.proto API
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
    @brief  DigitalIOController constructor
*/
DigitalIOController::DigitalIOController() {
  _dio_model = new DigitalIOModel();
  _dio_hardware = new DigitalIOHardware();
  // Set the default maximum number of digital pins to 0
  // NOTE: This will be set during runtime by the CheckinResponse message
  SetMaxDigitalPins(0);
}

/*!
    @brief  DigitalIOController destructor
*/
DigitalIOController::~DigitalIOController() {
  delete _dio_model;
  delete _dio_hardware;
}

/*!
    @brief  Set the maximum number of digital pins
    @param  max_digital_pins
            The maximum number of digital pins
*/
void DigitalIOController::SetMaxDigitalPins(uint8_t max_digital_pins) {
  _max_digitalio_pins = max_digital_pins;
}

/*!
    @brief  Routes messages using the digitalio.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool DigitalIOController::Router(pb_istream_t *stream) {
  // Attempt to decode the DigitalIO B2D envelope
  ws_digitalio_B2D b2d = ws_digitalio_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_digitalio_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN(
        "[digitalio] ERROR: Unable to decode DigitalIO B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_digitalio_B2D_add_tag:
    res = Handle_DigitalIO_Add(&b2d.payload.add);
    break;
  case ws_digitalio_B2D_remove_tag:
    // TODO: Re-implement handler using message instead of stream
    // res = Handle_DigitalIO_Remove(&b2d.payload.remove);
    WS_DEBUG_PRINTLN("[digitalio] WARNING: Remove handler not implemented yet");
    break;
  case ws_digitalio_B2D_write_tag:
    // TODO: Re-implement handler using message instead of stream
    // res = Handle_DigitalIO_Write(&b2d.payload.write);
    WS_DEBUG_PRINTLN("[digitalio] WARNING: Write handler not implemented yet");
    break;
  default:
    WS_DEBUG_PRINTLN("[digitalio] WARNING: Unsupported DigitalIO payload");
    res = false;
    break;
  }

  return res;
}

// TODO: Do we even need this function?
bool DigitalIOController::Handle_DigitalIO_Add(ws_digitalio_Add *msg) {
  WS_DEBUG_PRINTLN("[digitalio] Handle_DigitalIO_Add MESSAGE...");
  // Strip the D/A prefix off the pin name and convert to a uint8_t pin number
  uint8_t pin_name = atoi(msg->pin_name + 1);

  // Check if the provided pin is also the status LED pin
  if (_dio_hardware->IsStatusLEDPin(pin_name))
    ReleaseStatusPixel();

  // Deinit the pin if it's already in use
  if (GetPinIdx(pin_name) != -1)
    _dio_hardware->deinit(pin_name);

  // Attempt to configure the pin
  if (!_dio_hardware->ConfigurePin(pin_name, msg->gpio_direction)) {
    WS_DEBUG_PRINTLN(
        "[digitalio] ERROR: Pin provided an invalid protobuf direction!");
    return false;
  }

  // Create the digital pin and add it to the vector
  DigitalIOPin new_pin = {
      .pin_name = pin_name,
      .pin_direction = msg->gpio_direction,
      .sample_mode = msg->sample_mode,
      .pin_value = msg->value,
      .pin_period = (ulong)(msg->period * 1000.0f),
      .prv_pin_time = 0 // Set to 0 so timer pins trigger immediately
  };
  _digitalio_pins.push_back(new_pin);

  // Print out the pin's details
  WS_DEBUG_PRINTLN("[digitalio] Added new pin:");
  WS_DEBUG_PRINT("Pin Name: ");
  WS_DEBUG_PRINTLN(new_pin.pin_name);
  WS_DEBUG_PRINT("Period: ");
  WS_DEBUG_PRINTLN(new_pin.pin_period);
  WS_DEBUG_PRINT("Sample Mode: ");
  WS_DEBUG_PRINTLN(new_pin.sample_mode);
  WS_DEBUG_PRINT("Direction: ");
  WS_DEBUG_PRINTLN(new_pin.pin_direction);

  return true;
}

/*!
    @brief  Removes a digital pin from the controller, if it exists
    @param  msg
            The DigitalIORemove message.
    @return True if the digital pin was successfully removed, False otherwise.
*/
bool DigitalIOController::Handle_DigitalIO_Remove(ws_digitalio_Remove *msg) {
  // Get the pin's name
  int pin_name = atoi(msg->pin_name + 1);

  // Bail out if the pin does not exist within controller
  if (GetPinIdx(pin_name) == -1) {
    WS_DEBUG_PRINTLN(
        "[digitalio] ERROR: Unable to find output pin on the controller!");
    return false;
  }

  // Deinitialize the pin
  _dio_hardware->deinit(pin_name);
  return true;
}

/*!
    @brief  Get the index of a digital output pin
    @param  pin_name
            The pin's name.
    @return The index of the digital output pin.
*/
int DigitalIOController::GetPinIdx(uint8_t pin_name) {
  for (int i = 0; i < _digitalio_pins.size(); i++) {
    if (_digitalio_pins[i].pin_name == pin_name) {
      return i;
    }
  }
  return -1; // Pin not found
}

/*!
    @brief  Write a digital pin
    @param  stream
            The nanopb input stream.
    @return True if the digital pin was successfully written.
*/
bool DigitalIOController::Handle_DigitalIO_Write(ws_digitalio_Write *msg) {
  // Get the digital pin
  int pin_idx = GetPinIdx(atoi(msg->pin_name + 1));
  // Check if the pin was found and is a valid digital output pin
  if (pin_idx == -1) {
    WS_DEBUG_PRINTLN(
        "[digitalio] ERROR: Unable to find the requested output pin!");
    return false;
  }

  // Ensure we got the correct value type
  if (!msg->value.which_value == ws_sensor_Event_bool_value_tag) {
    WS_DEBUG_PRINTLN("[digitalio] ERROR: controller got invalid value type!");
    return false;
  }

  WS_DEBUG_PRINT("[digitalio] Writing value: ");
  WS_DEBUG_PRINTLN(msg->value.value.bool_value);
  WS_DEBUG_PRINT("on Pin: ");
  WS_DEBUG_PRINTLN(_digitalio_pins[pin_idx].pin_name);

  // Is the pin already set to this value? If so, we don't need to write it
  // again
  if (_digitalio_pins[pin_idx].pin_value == msg->value.value.bool_value)
    return true;

  // Call hardware to write the value type
  _dio_hardware->SetValue(_digitalio_pins[pin_idx].pin_name,
                          msg->value.value.bool_value);

  // Update the pin's value
  _digitalio_pins[pin_idx].pin_value = msg->value.value.bool_value;

  return true;
}

/*!
    @brief  Check if a pin's timer has expired
    @param  pin
            The pin to check.
    @param  cur_time
            The current time.
    @return True if the pin's timer has expired.
*/
bool DigitalIOController::IsPinTimerExpired(DigitalIOPin *pin, ulong cur_time) {
  return cur_time - pin->prv_pin_time > pin->pin_period;
}

/*!
    @brief  Print a pin's ID and value
    @param  pin
            The specified pin.
*/
void DigitalIOController::PrintPinValue(DigitalIOPin *pin) {
  if (WsV2._sdCardV2->isModeOffline())
    return;
  WS_DEBUG_PRINT("[digitalio] DIO Pin D");
  WS_DEBUG_PRINT(pin->pin_name);
  WS_DEBUG_PRINT(" | value: ");
  WS_DEBUG_PRINTLN(pin->prv_pin_value);
}

/*!
    @brief  Check if a pin's timer has expired
    @param  pin
            The pin to check.
    @return True if the pin's timer has expired.
*/
bool DigitalIOController::CheckTimerPin(DigitalIOPin *pin) {
  ulong cur_time = millis();
  // Bail out if the pin's timer has not expired
  if (!IsPinTimerExpired(pin, cur_time))
    return false;

  // Fill in the pin's current time and value
  pin->prv_pin_time = cur_time;
  pin->pin_value = _dio_hardware->GetValue(pin->pin_name);

  PrintPinValue(pin);
  return true;
}

/*!
    @brief  Check if a pin's value has changed
    @param  pin
            The pin to check.
    @return True if the pin's value has changed.
*/
bool DigitalIOController::CheckEventPin(DigitalIOPin *pin) {
  // Get the pin's current value
  pin->pin_value = _dio_hardware->GetValue(pin->pin_name);
  // Bail out if the pin value hasn't changed
  if (pin->pin_value == pin->prv_pin_value)
    return false;
  // Update the pin's previous value to the current value
  pin->prv_pin_value = pin->pin_value;

  PrintPinValue(pin);
  return true;
}

/*!
    @brief  Encode and publish a pin event
    @param  pin_name
            The pin's name.
    @param  pin_value
            The pin's value.
    @return True if the pin event was successfully encoded and published.
*/
bool DigitalIOController::EncodePublishPinEvent(uint8_t pin_name,
                                                bool pin_value) {
  // Prefix pin_name with "D" to match the expected pin name format
  char c_pin_name[12];
  sprintf(c_pin_name, "D%d", pin_name);

  // If we are in ONLINE mode, publish the event to the broker
  if (!WsV2._sdCardV2->isModeOffline()) {
    WS_DEBUG_PRINT(
        "[digitalio] Publishing DigitalIOEvent message to broker for pin: ");
    WS_DEBUG_PRINTLN(c_pin_name);
    // Encode the DigitalIOEvent message
    if (!_dio_model->EncodeDigitalIOEvent(c_pin_name, pin_value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode DigitalIOEvent message!");
      return false;
    }

    // Publish the DigitalIOEvent message to the broker
    if (!WsV2.PublishD2b(ws_signal_DeviceToBroker_digitalio_tag,
                         _dio_model->GetDigitalIOEventMsg())) {
      WS_DEBUG_PRINTLN("[digitalio] ERROR: Unable to publish event message, "
                       "moving onto the next pin!");
      return false;
    }
    WS_DEBUG_PRINTLN("[digitalio] Published DigitalIOEvent to broker!")
  } else {
    // let's log the event to the SD card
    if (!WsV2._sdCardV2->LogGPIOSensorEventToSD(pin_name, pin_value,
                                                ws_sensor_Type_T_BOOLEAN))
      return false;
  }

  return true;
}

/*!
    @brief  Iterates through the digital pins and updates their values
      (if necessary) and publishes the event to the broker.
*/
void DigitalIOController::Update() {
  // Bail out if we have no digital pins to poll
  if (_digitalio_pins.empty())
    return;

  for (int i = 0; i < _digitalio_pins.size(); i++) {
    // Create a pin object for this iteration
    DigitalIOPin &pin = _digitalio_pins[i];
    // Skip if the pin is an output
    if (pin.pin_direction == ws_digitalio_Direction_D_OUTPUT)
      continue;

    if (pin.sample_mode == ws_digitalio_SampleMode_SM_EVENT) {
      // Check if the pin value has changed
      if (!CheckEventPin(&pin))
        continue; // No change in pin value detected, move onto the next pin

      // Encode and publish the event
      if (!EncodePublishPinEvent(pin.pin_name, pin.pin_value)) {
        WS_DEBUG_PRINTLN("[digitalio] ERROR: Unable to record pin value!");
        continue;
      }
    } else if (pin.sample_mode == ws_digitalio_SampleMode_SM_TIMER) {
      // Check if the timer has expired
      if (!CheckTimerPin(&pin))
        continue; // Timer has not expired yet, move onto the next pin

      // Encode and publish the event
      if (!EncodePublishPinEvent(pin.pin_name, pin.pin_value)) {
        WS_DEBUG_PRINTLN("[digitalio] ERROR: Unable to record pin value!");
        continue;
      }
    } else {
      // Invalid sample mode
      WS_DEBUG_PRINT("[digitalio] ERROR: DigitalIO Pin ");
      WS_DEBUG_PRINT(pin.pin_name);
      WS_DEBUG_PRINTLN(" contains an invalid sample mode!");
    }
  }
}