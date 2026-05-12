/*!
 * @file src/components/digitalIO/controller.cpp
 *
 * Controller for the digitalio.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"
#include "../expander/controller.h"
#include "hardware.h"

/*!
    @brief  DigitalIOController constructor
*/
DigitalIOController::DigitalIOController() {
  _dio_model = new DigitalIOModel();
}

/*!
    @brief  DigitalIOController destructor
*/
DigitalIOController::~DigitalIOController() {
  for (size_t i = 0; i < _pins_input.size(); i++)
    delete _pins_input[i];
  for (size_t i = 0; i < _pins_output.size(); i++)
    delete _pins_output[i];
  delete _dio_model;
}

/*!
    @brief  Set the maximum number of digital pins
    @param  max_digital_pins
            The maximum number of digital pins
*/
void DigitalIOController::SetMaxDigitalPins(uint8_t max_digital_pins) {
  _pins_input.reserve(max_digital_pins);
  _pins_output.reserve(max_digital_pins);
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
    WS_DEBUG_PRINTLN("[dio] ERROR: Unable to decode DigitalIO B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_digitalio_B2D_add_tag:
    res = Handle_DigitalIO_Add(&b2d.payload.add);
    break;
  case ws_digitalio_B2D_write_tag:
    res = Handle_DigitalIO_Write(&b2d.payload.write);
    break;
  case ws_digitalio_B2D_remove_tag:
    res = Handle_DigitalIO_Remove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[dio] WARNING: Unsupported DigitalIO payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Removes a pin from both vectors by pin number.
            Deletes the pin object (destructor deinits hardware).
    @param  pin_num
            The pin number to remove.
    @return True if the pin was found and removed.
*/
bool DigitalIOController::RemovePin(uint8_t pin_num,
                                    ExpanderHardware *expander) {
  for (size_t i = 0; i < _pins_output.size(); i++) {
    if (_pins_output[i]->GetPinNum() == pin_num &&
        _pins_output[i]->GetExpanderDriver() == expander) {
      delete _pins_output[i];
      _pins_output.erase(_pins_output.begin() + i);
      return true;
    }
  }
  for (size_t i = 0; i < _pins_input.size(); i++) {
    if (_pins_input[i]->GetPinNum() == pin_num &&
        _pins_input[i]->GetExpanderDriver() == expander) {
      delete _pins_input[i];
      _pins_input.erase(_pins_input.begin() + i);
      return true;
    }
  }
  return false;
}

/*!
    @brief  Adds a digital pin to the controller
    @param  msg
            The DigitalIOAdd message.
    @return True if the digital pin was successfully added, False otherwise.
*/
bool DigitalIOController::Handle_DigitalIO_Add(ws_digitalio_Add *msg) {
  WS_DEBUG_PRINTLN("[dio] Handle_DigitalIO_Add MESSAGE...");
  uint8_t pin_num = 0;

  // Validate all fields of the digital pin add message before adding the pin
  if (msg->gpio_direction == ws_digitalio_Direction_D_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Invalid GPIO direction specified!");
    return false;
  }
  if (msg->sample_mode == ws_digitalio_SampleMode_SM_UNSPECIFIED) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Invalid sample mode specified!");
    return false;
  }
  if (msg->sample_mode == ws_digitalio_SampleMode_SM_TIMER &&
      msg->period <= 0) {
    WS_DEBUG_PRINTLN(
        "[dio] ERROR: Invalid period specified for timer sample mode!");
    return false;
  }

  if (!ExpanderHardware::ParsePinNum(msg->pin_name, pin_num)) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Malformed expander pin name!");
    return false;
  }

  // Resolve the expander driver if this is an expander pin
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin_name, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin_name + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      WS_DEBUG_PRINTLN("[dio] ERROR: Expander not found for address!");
      return false;
    }
  }

  // Remove existing pin if re-adding (destructor deinits hardware)
  RemovePin(pin_num, expander_drv);

  // Get the initial value from the write message (if present)
  bool initial_value = false;
  if (msg->has_write && msg->write.has_value) {
    initial_value = msg->write.value.value.bool_value;
  }

  // Initialize a new digital pin instance
  DigitalIOHardware *new_pin = new DigitalIOHardware(
      pin_num, msg->gpio_direction, msg->sample_mode, initial_value,
      (ulong)(msg->period * 1000.0f), expander_drv);

  // Add the pin to the controller's list of pins
  if (msg->gpio_direction == ws_digitalio_Direction_D_INPUT ||
      msg->gpio_direction == ws_digitalio_Direction_D_INPUT_PULL_UP) {
    _pins_input.push_back(new_pin);
  } else if (msg->gpio_direction == ws_digitalio_Direction_D_OUTPUT) {
    _pins_output.push_back(new_pin);
  }

  WS_DEBUG_PRINTLN("[dio] Added new pin:");
  WS_DEBUG_PRINT("Pin Name: ");
  WS_DEBUG_PRINTLNVAR(new_pin->GetPinNum());
  WS_DEBUG_PRINT("Direction: ");
  WS_DEBUG_PRINTLNVAR(new_pin->GetDirection());

  return true;
}

/*!
    @brief  Removes a digital pin from the controller, if it exists
    @param  msg
            The DigitalIORemove message.
    @return True if the digital pin was successfully removed, False otherwise.
*/
bool DigitalIOController::Handle_DigitalIO_Remove(ws_digitalio_Remove *msg) {
  uint8_t pin_num = 0;
  if (!ExpanderHardware::ParsePinNum(msg->pin_name, pin_num)) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Malformed expander pin name!");
    return false;
  }

  // Resolve the expander driver if this is an expander pin
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin_name, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin_name + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      WS_DEBUG_PRINTLN("[dio] ERROR: Expander not found for address!");
      return false;
    }
  }

  if (!RemovePin(pin_num, expander_drv)) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Unable to find requested pin!");
    return false;
  }

  WS_DEBUG_PRINT("[dio] Pin removed: ");
  WS_DEBUG_PRINTLNVAR(pin_num);
  return true;
}

/*!
    @brief  Get a pointer to a digital pin by pin number
    @param  pin_num
            The pin's number.
    @return Pointer to the digital pin, or nullptr if not found.
*/
DigitalIOHardware *DigitalIOController::GetPin(uint8_t pin_num,
                                               ExpanderHardware *expander) {
  for (size_t i = 0; i < _pins_output.size(); i++) {
    if (_pins_output[i]->GetPinNum() == pin_num &&
        _pins_output[i]->GetExpanderDriver() == expander)
      return _pins_output[i];
  }
  for (size_t i = 0; i < _pins_input.size(); i++) {
    if (_pins_input[i]->GetPinNum() == pin_num &&
        _pins_input[i]->GetExpanderDriver() == expander)
      return _pins_input[i];
  }
  return nullptr;
}

/*!
    @brief  Write a digital pin
    @param  msg
            Pointer to the DigitalIO write message.
    @return True if the digital pin was successfully written.
*/
bool DigitalIOController::Handle_DigitalIO_Write(ws_digitalio_Write *msg) {
  uint8_t pin_num = 0;
  if (!ExpanderHardware::ParsePinNum(msg->pin_name, pin_num)) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Malformed expander pin name!");
    return false;
  }

  // Resolve the expander driver if this is an expander pin
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin_name, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin_name + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      WS_DEBUG_PRINTLN("[dio] ERROR: Expander not found for address!");
      return false;
    }
  }

  DigitalIOHardware *pin = GetPin(pin_num, expander_drv);
  if (!pin) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Unable to find the requested output pin!");
    return false;
  }

  if (pin->GetDirection() != ws_digitalio_Direction_D_OUTPUT) {
    WS_DEBUG_PRINTLN("[dio] ERROR: Requested pin is not a digital output pin!");
    return false;
  }

  // Ensure the value type to write is boolean
  if (msg->value.which_value != ws_sensor_Event_bool_value_tag) {
    WS_DEBUG_PRINTLN("[dio] ERROR: controller received invalid value type!");
    return false;
  }

  WS_DEBUG_PRINT("[dio] Writing: ");
  WS_DEBUG_PRINTVAR(msg->value.value.bool_value);
  WS_DEBUG_PRINT(" to Pin ");
  WS_DEBUG_PRINTLNVAR(pin->GetPinNum());

  pin->Write(msg->value.value.bool_value);
  return true;
}

/*!
    @brief  Encode and publish a pin event
    @param  pin
            Pointer to the digital pin hardware object.
    @return True if the pin event was successfully encoded and published.
*/
bool DigitalIOController::EncodePublishPinEvent(DigitalIOHardware *pin) {
  uint8_t pin_num = pin->GetPinNum();
  bool pin_value = pin->GetPinValue();

  // Format pin name: expander pins use "EXP_0xNN_P", native pins use "DN"
  char c_pin_name[20];
  ExpanderHardware *expander = pin->GetExpanderDriver();
  if (expander != nullptr) {
    ExpanderHardware::FormatPinName(c_pin_name, sizeof(c_pin_name),
                                    expander->getAddress(), pin_num);
  } else {
    snprintf(c_pin_name, sizeof(c_pin_name), "D%d", pin_num);
  }

  if (!Ws._sdCardV2->isModeOffline()) {
    WS_DEBUG_PRINT("[dio] Publish Event: ");
    WS_DEBUG_PRINTVAR(c_pin_name);
    WS_DEBUG_PRINT(" | value: ");
    WS_DEBUG_PRINTLNVAR(pin_value);

    if (!_dio_model->EncodeDigitalIOEvent(c_pin_name, pin_value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode DigitalIOEvent message!");
      return false;
    }

    if (!Ws.PublishD2b(ws_signal_DeviceToBroker_digitalio_tag,
                       _dio_model->GetDigitalIOD2B())) {
      WS_DEBUG_PRINTLN("[dio] ERROR: Unable to publish event message, "
                       "moving onto the next pin!");
      return false;
    }
    WS_DEBUG_PRINTLN("[dio] Published!")
  } else {
    if (!Ws._sdCardV2->LogGPIOSensorEventToSD(pin_num, pin_value,
                                              ws_sensor_Type_T_BOOLEAN))
      return false;
  }

  return true;
}

/*!
    @brief  Iterates through the digital pins and updates their values
      (if necessary) and publishes the event to the broker.
    @param  force
            If true, forces a read on all pins regardless of timers/events.
*/
void DigitalIOController::update(bool force) {
  if (_pins_input.empty())
    return;

  for (size_t i = 0; i < _pins_input.size(); i++) {
    DigitalIOHardware *pin = _pins_input[i];

    if (force) {
      if (pin->DidReadSend())
        continue;
      pin->ReadValue();
    } else {
      bool changed;
      if (pin->GetSampleMode() == ws_digitalio_SampleMode_SM_EVENT)
        changed = pin->CheckEvent();
      else
        changed = pin->CheckTimer();
      if (!changed)
        continue;
    }

    if (!EncodePublishPinEvent(pin)) {
      WS_DEBUG_PRINTLN("[dio] ERROR: Unable to record pin value!");
      pin->ResetSendFlag();
      continue;
    }
    pin->MarkSent();
  }
}

/*!
    @brief  SLEEP MODE: Checks if all digital input pins have been read and
   their values sent.
    @return True if all pins have been read and sent, False otherwise.
*/
bool DigitalIOController::UpdateComplete() {
  for (size_t i = 0; i < _pins_input.size(); i++) {
    if (!_pins_input[i]->DidReadSend())
      return false;
  }
  return true;
}

/*!
    @brief  SLEEP MODE: Resets all digital input pins' did_read_send flags to
   false.
*/
void DigitalIOController::ResetFlags() {
  for (size_t i = 0; i < _pins_input.size(); i++) {
    _pins_input[i]->ResetSendFlag();
  }
}
