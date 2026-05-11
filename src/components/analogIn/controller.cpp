/*!
 * @file src/components/analogIn/controller.cpp
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

/*!
    @brief  AnalogIn controller constructor
*/
AnalogInController::AnalogInController() {
  _analogin_model = new AnalogInModel();
  _mcu_vref = DEFAULT_MCU_VREF;
}

/*!
    @brief  AnalogIn controller destructor
*/
AnalogInController::~AnalogInController() {
  for (size_t i = 0; i < _pins.size(); i++)
    delete _pins[i];
  delete _analogin_model;
}

/*!
    @brief  Set the reference voltage for the analog pins
    @param  voltage
            The reference voltage.
*/
void AnalogInController::SetRefVoltage(float voltage) { _mcu_vref = voltage; }

/*!
    @brief  Allocate memory for the total number of analog pins.
    @param  max_analog_pins
            The hardware's maximum number of analog pins.
*/
void AnalogInController::SetMaxAnalogPins(uint8_t max_analog_pins) {
  _pins.reserve(max_analog_pins);
}

/*!
    @brief  Routes messages using the analogin.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool AnalogInController::Router(pb_istream_t *stream) {
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
bool AnalogInController::RemovePin(uint8_t pin_num) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->GetPinNum() == pin_num) {
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
AnalogInHardware *AnalogInController::GetPin(uint8_t pin_num) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->GetPinNum() == pin_num)
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
bool AnalogInController::Handle_AnalogInAdd(ws_analogin_Add *msg) {
  WS_DEBUG_PRINTLN("[analogin] Handle_AnalogInAdd MESSAGE...");
  uint8_t pin_num = 0;

  // Check if the pin is located on an expander and resolve the expander driver
  // Expander Pin Format: "EXP_<EXPANDER-I2C-ADDR>_<PIN-#>"
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin_name, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin_name + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      WS_DEBUG_PRINTLN("[analogin] ERROR: Expander not found for address!");
      return false;
    }
    const char *pin_str = strchr(msg->pin_name + 4, '_');
    if (!pin_str) {
      WS_DEBUG_PRINTLN("[analogin] ERROR: Malformed expander pin name!");
      return false;
    }
    pin_num = atoi(pin_str + 1);
  } else {
    pin_num = atoi(msg->pin_name + 1);
  }

  // If pin is being updated, remove the existing pin first
  if (!RemovePin(pin_num)) {
    WS_DEBUG_PRINT("[analogin] ERROR: Unable to find requested pin: ");
    WS_DEBUG_PRINTLNVAR(msg->pin_name);
    return false;
  }

  // Create a new analog input pin
  AnalogInHardware *new_pin = new AnalogInHardware(
      pin_num, msg->read_mode, msg->sample_mode, (ulong)(msg->period * 1000.0f),
      _mcu_vref, expander_drv);

  // Add the pin to the controller's list
  _pins.push_back(new_pin);

  // Print out the pin's details
  WS_DEBUG_PRINTLN("[analogin] Added new pin:");
  WS_DEBUG_PRINT("Pin Name: ");
  WS_DEBUG_PRINTLNVAR(new_pin->GetPinNum());
  WS_DEBUG_PRINT("Period: ");
  WS_DEBUG_PRINTLN(msg->period * 1000.0f);
  WS_DEBUG_PRINT("Read Mode: ");
  ws_sensor_Type pin_read_mode = new_pin->GetReadMode();
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
bool AnalogInController::Handle_AnalogInRemove(ws_analogin_Remove *msg) {
  // Get the pin name
  uint8_t pin_num = atoi(msg->pin_name + 1);

  if (!RemovePin(pin_num)) {
    WS_DEBUG_PRINTLN("[analogin] ERROR: Unable to find requested pin!");
    return false;
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
bool AnalogInController::EncodePublishPinEvent(AnalogInHardware *pin) {
  uint8_t pin_num = pin->GetPinNum();
  float value = pin->GetValue();
  ws_sensor_Type read_type = pin->GetReadMode();

  if (Ws._sdCardV2->isModeOffline()) {
    return Ws._sdCardV2->LogGPIOSensorEventToSD(pin_num, value, read_type);
  }

  if (read_type == ws_sensor_Type_T_RAW) {
    if (!_analogin_model->EncodeAnalogInEventRaw(pin_num, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIn raw adc message!");
      return false;
    }
  } else if (read_type == ws_sensor_Type_T_VOLTAGE) {
    if (!_analogin_model->EncodeAnalogInEventVoltage(pin_num, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIn voltage message!");
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid read type for AnalogInEvent message!");
    return false;
  }

  // Publish the AnalogIn message to the broker
  WS_DEBUG_PRINT("Publishing AnalogInEvent...");
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_analogin_tag,
                     _analogin_model->GetAnalogInD2B())) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish analogin voltage event message, "
                     "moving onto the next pin!");
    return false;
  }
  WS_DEBUG_PRINTLN("Published!");

  return true;
}

/*!
    @brief  Update/polling loop for the AnalogIn controller.
    @param  force
            If true, forces a read on all pins regardless of period.
*/
void AnalogInController::update(bool force) {
  // Bail-out if the vector is empty
  if (_pins.empty())
    return;

  for (size_t i = 0; i < _pins.size(); i++) {
    AnalogInHardware *pin = _pins[i];

    // Is the pin ready for a new reading?
    if (!force) {
      bool ready;
      if (pin->GetSampleMode() == ws_analogin_SampleMode_SM_EVENT) {
        ready = pin->CheckEvent();
      } else {
        ready = pin->CheckTimer();
      }
      if (!ready)
        continue;
    } else {
      // Sleep wake - force a new reading
      if (pin->DidReadSend())
        continue;
      pin->ReadValue();
    }

    if (!EncodePublishPinEvent(pin)) {
      WS_DEBUG_PRINTLN("[analogin] ERROR: Unable to record pin value!");
      pin->ResetSendFlag();
      continue;
    }
    pin->MarkSent();
  }
}

/*!
    @brief  Checks if all analog pins have been read and their values sent.
    @return True if all pins have been read and sent, False otherwise.
*/
bool AnalogInController::UpdateComplete() {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (!_pins[i]->DidReadSend()) {
      return false;
    }
  }
  return true;
}

/*!
    @brief  Resets all analog pins' did_read_send flags to false.
*/
void AnalogInController::ResetFlags() {
  for (size_t i = 0; i < _pins.size(); i++) {
    _pins[i]->ResetSendFlag();
  }
}
