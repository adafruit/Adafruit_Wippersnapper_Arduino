/*!
 * @file src/components/analogIO/controller.cpp
 *
 * Controller for the analogin.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"
#include "../expander/controller.h"
#include "hardware.h"

namespace {
bool reportPinError(const char *pin_name, const char *error_msg) {
  if (Ws->_sdCardV2->isModeOffline()) {
    WS_DEBUG_PRINT("[analogin] ERROR on ");
    WS_DEBUG_PRINT(pin_name);
    WS_DEBUG_PRINT(": ");
    WS_DEBUG_PRINTLN(error_msg);
    return false;
  }

  Ws->error_handler->publishComponentError(pin_name, error_msg);
  return false;
}
} // namespace

/*!
    @brief  AnalogIO controller constructor
*/
AnalogIOController::AnalogIOController() {
  _analogin_model = new AnalogIOModel();
  _mcu_ref_voltage = DEFAULT_MCU_VREF;
}

/*!
    @brief  AnalogIO controller destructor
*/
AnalogIOController::~AnalogIOController() {
  for (size_t i = 0; i < _pins.size(); i++)
    delete _pins[i];
  delete _analogin_model;
}

/*!
    @brief  Set the reference voltage for the analog pins
    @param  voltage
            The reference voltage.
*/
void AnalogIOController::SetRefVoltage(float voltage) {
  _mcu_ref_voltage = voltage;
}

/*!
    @brief  Allocate memory for the total number of analog pins.
    @param  max_analog_pins
            The hardware's maximum number of analog pins.
*/
void AnalogIOController::SetMaxAnalogPins(uint8_t max_analog_pins) {
  _pins.reserve(max_analog_pins);
}

/*!
    @brief  Routes messages using the analogin.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool AnalogIOController::Router(pb_istream_t *stream) {
  // Attempt to decode the AnalogIn B2D envelope
  ws_analogin_B2D b2d = ws_analogin_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_analogin_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN(
        "[analogin] ERROR: Unable to decode AnalogIn B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_analogin_B2D_add_tag:
    res = Handle_AnalogInAdd(&b2d.payload.add);
    break;
  case ws_analogin_B2D_remove_tag:
    res = Handle_AnalogInRemove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[analogin] WARNING: Unsupported AnalogIn payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Removes a pin from the vector by pin number.
            Deletes the pin object (destructor deinits hardware).
    @param  pin_num
            The pin number to remove.
    @return True if the pin was found and removed.
*/
bool AnalogIOController::RemovePin(uint8_t pin_num,
                                   ExpanderHardware *expander) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->getPinNum() == pin_num &&
        _pins[i]->getExpander() == expander) {
      delete _pins[i];
      _pins.erase(_pins.begin() + i);
      return true;
    }
  }
  return false;
}

/*!
    @brief  Get a pointer to an analog pin by pin number.
    @param  pin_num
            The pin's number.
    @return Pointer to the analog pin, or nullptr if not found.
*/
AnalogIOHardware *AnalogIOController::GetPin(uint8_t pin_num,
                                             ExpanderHardware *expander) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->getPinNum() == pin_num && _pins[i]->getExpander() == expander)
      return _pins[i];
  }
  return nullptr;
}

/*!
    @brief  Handles an AnalogInAdd message from the broker and adds a
            new analog pin to the controller.
    @param  msg
            The AnalogInAdd message.
    @return True if the pin was successfully added, False otherwise.
*/
bool AnalogIOController::Handle_AnalogInAdd(ws_analogin_Add *msg) {
  WS_DEBUG_PRINTLN("[analogin] Handle_AnalogInAdd MESSAGE...");
  uint8_t pin_num = 0;
  ExpanderHardware *expander_drv = nullptr;
  if (!Ws->_expander_controller->ResolvePinName(msg->pin_name, pin_num,
                                                &expander_drv)) {
    return reportPinError(msg->pin_name, "Unable to resolve pin name");
  }

  // Validate the read mode
  if (msg->read_mode != ws_sensor_Type_T_RAW &&
      msg->read_mode != ws_sensor_Type_T_VOLTAGE) {
    return reportPinError(msg->pin_name, "Invalid read mode");
  }
  // Validate the sample mode
  if (msg->sample_mode != ws_analogin_SampleMode_SM_TIMER &&
      msg->sample_mode != ws_analogin_SampleMode_SM_EVENT) {
    return reportPinError(msg->pin_name, "Invalid sample mode");
  }

  // If pin is being updated, remove the existing pin first
  RemovePin(pin_num, expander_drv);

  // Native MCU pins use the MCU's reference voltage; expander ADC pins carry
  // their own reference voltage in the message.
  float ref_voltage = _mcu_ref_voltage;
  if (expander_drv != nullptr) {
    ref_voltage = msg->ref_voltage;
  }

  // Create a new analog input pin
  AnalogIOHardware *new_pin = new AnalogIOHardware(
      msg->pin_name, pin_num, msg->read_mode, msg->sample_mode,
      (ulong)(msg->period * 1000.0f), ref_voltage, expander_drv);

  // Add the pin to the controller's list
  _pins.push_back(new_pin);

  // Print out the pin's details
  WS_DEBUG_PRINTLN("[analogin] Added new pin:");
  WS_DEBUG_PRINT("Pin Name: ");
  WS_DEBUG_PRINTLNVAR(new_pin->getPinName());
  WS_DEBUG_PRINT("Period: ");
  WS_DEBUG_PRINTLNVAR(msg->period * 1000.0f);
  WS_DEBUG_PRINT("Read Mode: ");
  ws_sensor_Type pin_read_mode = new_pin->getReadMode();
  WS_DEBUG_PRINTLNVAR(pin_read_mode);

  return true;
}

/*!
    @brief  Handles an AnalogInRemove message from the broker and removes
            the requested analog pin from the controller.
    @param  msg
            The AnalogInRemove message.
    @return True if the pin was successfully removed, False otherwise.
*/
bool AnalogIOController::Handle_AnalogInRemove(ws_analogin_Remove *msg) {
  uint8_t pin_num = 0;
  ExpanderHardware *expander_drv = nullptr;
  if (!Ws->_expander_controller->ResolvePinName(msg->pin_name, pin_num,
                                                &expander_drv)) {
    return reportPinError(msg->pin_name, "Unable to resolve pin name");
  }

  if (!RemovePin(pin_num, expander_drv)) {
    return reportPinError(msg->pin_name, "Failed to find pin");
  }

  WS_DEBUG_PRINT("[analogin] Removed pin: ");
  WS_DEBUG_PRINTLNVAR(msg->pin_name);
  return true;
}

/*!
    @brief  Encodes and publishes an AnalogInEvent message to the broker
            or logs to SD card if offline.
    @param  pin
            Pointer to the analog pin hardware object.
    @return True if the message was successfully recorded.
*/
bool AnalogIOController::EncodePublishPinEvent(AnalogIOHardware *pin) {
  float value = pin->getValue();
  ws_sensor_Type read_type = pin->getReadMode();
  uint8_t pin_num = pin->getPinNum();
  char c_pin_name[20];
  ExpanderHardware *expander = pin->getExpander();
  if (expander != nullptr) {
    ExpanderHardware::FormatPinName(c_pin_name, sizeof(c_pin_name),
                                    expander->getAddress(), pin_num);
  } else {
    snprintf(c_pin_name, sizeof(c_pin_name), "A%d", pin_num);
  }

  if (Ws->_sdCardV2->isModeOffline()) {
    return Ws->_sdCardV2->LogGPIOSensorEventToSD(c_pin_name, value, read_type);
  }

  if (read_type == ws_sensor_Type_T_RAW) {
    if (!_analogin_model->encodeAnalogInEventRaw(c_pin_name, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIn raw adc message!");
      return false;
    }
  } else if (read_type == ws_sensor_Type_T_VOLTAGE) {
    if (!_analogin_model->encodeAnalogInEventVoltage(c_pin_name, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIn voltage message!");
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid read type for AnalogInEvent message!");
    return false;
  }

  // Publish the AnalogIn message to the broker
  WS_DEBUG_PRINT("Publishing AnalogInEvent...");
  if (!Ws->PublishD2b(ws_signal_DeviceToBroker_analogin_tag,
                      _analogin_model->getAnalogInD2b())) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish analogin voltage event message, "
                     "moving onto the next pin!");
    return false;
  }
  WS_DEBUG_PRINTLN("Published!");

  return true;
}

/*!
    @brief  Update/polling loop for the AnalogIO controller.
    @param  force
            If true, forces a read on all pins regardless of period.
*/
void AnalogIOController::update(bool force) {
  // Bail-out if the vector is empty
  if (_pins.empty())
    return;

  for (size_t i = 0; i < _pins.size(); i++) {
    AnalogIOHardware *pin = _pins[i];

    // Is the pin ready for a new reading?
    if (!force) {
      bool ready;
      if (pin->getSampleMode() == ws_analogin_SampleMode_SM_EVENT) {
        ready = pin->checkEvent();
      } else {
        ready = pin->checkTimer();
      }
      if (!ready)
        continue;
    } else {
      // Sleep wake - force a new reading
      if (pin->didReadSend())
        continue;
      pin->readValue();
    }

    if (!EncodePublishPinEvent(pin)) {
      reportPinError(pin->getPinName(), "Unable to record pin value!");
      pin->resetSendFlag();
      continue;
    }
    pin->markSent();
  }
}

/*!
    @brief  Checks if all analog pins have been read and their values sent.
    @return True if all pins have been read and sent, False otherwise.
*/
bool AnalogIOController::UpdateComplete() {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (!_pins[i]->didReadSend()) {
      return false;
    }
  }
  return true;
}

/*!
    @brief  Resets all analog pins' did_read_send flags to false.
*/
void AnalogIOController::ResetFlags() {
  for (size_t i = 0; i < _pins.size(); i++) {
    _pins[i]->resetSendFlag();
  }
}
