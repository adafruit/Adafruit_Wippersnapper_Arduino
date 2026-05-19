/*!
 * @file src/components/servo/controller.cpp
 *
 * Controller for the servo API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "controller.h"

/*!
    @brief  Constructor
*/
ServoController::ServoController() {
  _servo_model = new ServoModel();
  _active_servo_pins = 0;
}

/*!
    @brief  Destructor
*/
ServoController::~ServoController() {
  // De-initialize all servos
  for (int i = 0; i < _active_servo_pins; i++) {
    if (_servo_hardware[i] != nullptr) {
      delete _servo_hardware[i];
      _servo_hardware[i] = nullptr;
    }
  }

  if (_servo_model != nullptr) {
    delete _servo_model;
    _servo_model = nullptr;
  }
}

/*!
    @brief  Routes messages using the servo.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool ServoController::Router(pb_istream_t *stream) {
  // Attempt to decode the Servo B2D envelope
  ws_servo_B2D b2d = ws_servo_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_servo_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[servo] ERROR: Unable to decode Servo B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_servo_B2D_add_tag:
    res = Handle_Servo_Add(&b2d.payload.add);
    break;
  case ws_servo_B2D_remove_tag:
    res = Handle_Servo_Remove(&b2d.payload.remove);
    break;
  case ws_servo_B2D_write_tag:
    res = Handle_Servo_Write(&b2d.payload.write);
    break;
  default:
    WS_DEBUG_PRINTLN("[servo] WARNING: Unsupported Servo payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles a ServoAdd message
    @param  msg
            The ServoAdd message
    @returns True if successful, False otherwise
*/
bool ServoController::Handle_Servo_Add(ws_servo_Add *msg) {
  if (_active_servo_pins >= MAX_SERVOS) {
    WS_DEBUG_PRINTLN("[servo] Error: Maximum number of servos reached!");
    Ws.error_controller->publishComponentError(msg->servo_pin, "Maximum number of servos reached");
    return false;
  }

  // Attempt to create a ServoHardware object for the specified pin and attach it
  uint8_t pin = atoi(msg->servo_pin + 1);
  _servo_hardware[_active_servo_pins] =
      new ServoHardware(pin, (int)msg->min_pulse_width,
                        (int)msg->max_pulse_width, (int)msg->freq);
  if (!_servo_hardware[_active_servo_pins]->ServoAttach()) {
    Ws.error_controller->publishComponentError(msg->servo_pin, "Failed to attach servo");
    delete _servo_hardware[_active_servo_pins];
    _servo_hardware[_active_servo_pins] = nullptr;
  }

  _servo_hardware[_active_servo_pins]->ServoWrite((int)msg->min_pulse_width);
  WS_DEBUG_PRINT("[servo] Servo attached to pin: ");
  WS_DEBUG_PRINTLNVAR(msg->servo_pin);
  _active_servo_pins++;

  return true;
}

/*!
    @brief  Handles a ServoWrite message
    @param  msg
            The ServoWrite message
    @returns True if successful, False otherwise
*/
bool ServoController::Handle_Servo_Write(ws_servo_Write *msg) {
  uint8_t pin = atoi(msg->servo_pin + 1);
  int servo_idx = GetServoIndex(pin);
  if (servo_idx == -1) {
    Ws.error_controller->publishComponentError(msg->servo_pin, "Failed to find pin");
    return false;
  }
  // Write the pulse width to the servo
  _servo_hardware[servo_idx]->ServoWrite(msg->pulse_width);
  return true;
}

/*!
    @brief  Handles a ServoRemove message
    @param  msg
            The ServoRemove message
    @returns True if successful, False otherwise
*/
bool ServoController::Handle_Servo_Remove(ws_servo_Remove *msg) {
  if (_active_servo_pins <= 0) {
    Ws.error_controller->publishComponentError(msg->servo_pin, "No active servos to remove");
    return false;
  }

  uint8_t pin = atoi(msg->servo_pin + 1);
  int servo_idx = GetServoIndex(pin);
  if (servo_idx == -1) {
    Ws.error_controller->publishComponentError(msg->servo_pin, "Failed to find pin");
    return false;
  }

  // The destructor of ServoHardware will handle proper detachment
  delete _servo_hardware[servo_idx];
  _servo_hardware[servo_idx] = nullptr;

  // Shift _active_servo_pins down
  for (int i = servo_idx; i < _active_servo_pins - 1; i++) {
    _servo_hardware[i] = _servo_hardware[i + 1];
    _servo_hardware[i + 1] = nullptr;
  }
  _servo_hardware[_active_servo_pins - 1] = nullptr;
  _active_servo_pins--;

  WS_DEBUG_PRINT("[servo] Servo removed from pin: ");
  WS_DEBUG_PRINTLNVAR(msg->servo_pin);
  return true;
}

/*!
    @brief  Gets the index of the servo hardware for a given pin
    @param  pin
            The pin number to search for
    @returns The index of the servo hardware for the given pin, or -1 if not found
*/
int ServoController::GetServoIndex(uint8_t pin) {
  for (int i = 0; i < _active_servo_pins; i++) {
    if (_servo_hardware[i]->GetPin() == pin) {
      return i;
    }
  }
  return -1;
}