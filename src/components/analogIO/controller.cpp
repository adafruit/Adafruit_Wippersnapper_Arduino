/*!
 * @file src/components/analogIO/controller.cpp
 *
 * Controller for the analogio.proto API
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
    @brief  AnalogIO controller constructor
*/
AnalogIOController::AnalogIOController() {
  _analogio_model = new AnalogIOModel();
  _mcu_vref = DEFAULT_MCU_VREF;
}

/*!
    @brief  AnalogIO controller destructor
*/
AnalogIOController::~AnalogIOController() {
  for (size_t i = 0; i < _pins.size(); i++)
    delete _pins[i];
  delete _analogio_model;
}

/*!
    @brief  Set the reference voltage for the analog pins
    @param  voltage
            The reference voltage.
*/
void AnalogIOController::SetRefVoltage(float voltage) { _mcu_vref = voltage; }

/*!
    @brief  Allocate memory for the total number of analog pins.
    @param  max_analog_pins
            The hardware's maximum number of analog pins.
*/
void AnalogIOController::SetMaxAnalogPins(uint8_t max_analog_pins) {
  _pins.reserve(max_analog_pins);
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
    @brief  Removes a pin from the vector by pin number.
            Deletes the pin object (destructor deinits hardware).
    @param  pin_num
            The pin number to remove.
    @return True if the pin was found and removed.
*/
bool AnalogIOController::RemovePin(uint8_t pin_num) {
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
AnalogIOHardware *AnalogIOController::GetPin(uint8_t pin_num) {
  for (size_t i = 0; i < _pins.size(); i++) {
    if (_pins[i]->GetPinNum() == pin_num)
      return _pins[i];
  }
  return nullptr;
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
  uint8_t pin_num = 0;

  // Check if the pin is located on an expander and resolve the expander driver
  // Expander Pin Format: "EXP_<EXPANDER-I2C-ADDR>_<PIN-#>"
  ExpanderHardware *expander_drv = nullptr;
  if (strncmp(msg->pin_name, "EXP_", 4) == 0) {
    uint8_t i2c_addr = (uint8_t)strtoul(msg->pin_name + 4, nullptr, 16);
    expander_drv = Ws._expander_controller->GetDriver(i2c_addr);
    if (!expander_drv) {
      WS_DEBUG_PRINTLN("[analogio] ERROR: Expander not found for address!");
      return false;
    }
    const char *pin_str = strchr(msg->pin_name + 4, '_');
    if (!pin_str) {
      WS_DEBUG_PRINTLN("[analogio] ERROR: Malformed expander pin name!");
      return false;
    }
    pin_num = atoi(pin_str + 1);
  } else {
    pin_num = atoi(msg->pin_name + 1);
  }

  // If pin is being updated, remove the existing pin first
  if (!RemovePin(pin_num)) {
    WS_DEBUG_PRINT("[analogio] ERROR: Unable to find requested pin: ");
    WS_DEBUG_PRINTLNVAR(msg->pin_name);
    return false;
  }

  // Create a new analog input pin
  AnalogIOHardware *new_pin = new AnalogIOHardware(
      pin_num, msg->read_mode, msg->sample_mode, (ulong)(msg->period * 1000.0f),
      _mcu_vref, expander_drv);

  // Add the pin to the controller's list
  _pins.push_back(new_pin);

  // Print out the pin's details
  WS_DEBUG_PRINTLN("[analogio] Added new pin:");
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
    @brief  Handles an AnalogIORemove message from the broker and removes
            the requested analog pin from the controller.
    @param  msg
            The AnalogIORemove message.
    @return True if the pin was successfully removed, False otherwise.
*/
bool AnalogIOController::Handle_AnalogIORemove(ws_analogio_Remove *msg) {
  // Get the pin name
  uint8_t pin_num = atoi(msg->pin_name + 1);

  if (!RemovePin(pin_num)) {
    WS_DEBUG_PRINTLN("[analogio] ERROR: Unable to find requested pin!");
    return false;
  }

  WS_DEBUG_PRINT("[analogio] Removed pin: ");
  WS_DEBUG_PRINTLNVAR(msg->pin_name);
  return true;
}

/*!
    @brief  Encodes and publishes an AnalogIOEvent message to the broker
            or logs to SD card if offline.
    @param  pin
            Pointer to the analog pin hardware object.
    @return True if the message was successfully recorded.
*/
bool AnalogIOController::EncodePublishPinEvent(AnalogIOHardware *pin) {
  uint8_t pin_num = pin->GetPinNum();
  float value = pin->GetValue();
  ws_sensor_Type read_type = pin->GetReadMode();

  if (Ws._sdCardV2->isModeOffline()) {
    return Ws._sdCardV2->LogGPIOSensorEventToSD(pin_num, value, read_type);
  }

  if (read_type == ws_sensor_Type_T_RAW) {
    if (!_analogio_model->EncodeAnalogIOEventRaw(pin_num, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIO raw adc message!");
      return false;
    }
  } else if (read_type == ws_sensor_Type_T_VOLTAGE) {
    if (!_analogio_model->EncodeAnalogIOEventVoltage(pin_num, value)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode AnalogIO voltage message!");
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Invalid read type for AnalogIOEvent message!");
    return false;
  }

  // Publish the AnalogIO message to the broker
  WS_DEBUG_PRINT("Publishing AnalogIOEvent...");
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_analogio_tag,
                     _analogio_model->GetAnalogIOD2B())) {
    WS_DEBUG_PRINTLN("ERROR: Unable to publish analogio voltage event message, "
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
      if (pin->GetSampleMode() == ws_analogio_SampleMode_SM_EVENT) {
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
      WS_DEBUG_PRINTLN("[analogio] ERROR: Unable to record pin value!");
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
bool AnalogIOController::UpdateComplete() {
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
void AnalogIOController::ResetFlags() {
  for (size_t i = 0; i < _pins.size(); i++) {
    _pins[i]->ResetSendFlag();
  }
}
