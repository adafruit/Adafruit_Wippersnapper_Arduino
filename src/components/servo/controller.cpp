/*!
 * @file src/components/servo/controller.cpp
 *
 * Controller for the servo API
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

/**************************************************************************/
/*!
    @brief  Constructor
*/
/**************************************************************************/
ServoController::ServoController() {
  _servo_model = new ServoModel();
  _active_servo_pins = 0;
}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
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

/**************************************************************************/
/*!
    @brief  Handles a ServoAdd message
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool ServoController::Handle_Servo_Add(pb_istream_t *stream) {
  if (!_servo_model->DecodeServoAdd(stream)) {
    WS_DEBUG_PRINTLN("[servo] Error: Failed to decode ServoAdd message!");
    return false;
  }

  if (_active_servo_pins >= MAX_SERVOS) {
    WS_DEBUG_PRINTLN("[servo] Error: Maximum number of servos reached!");
    return false;
  }

  wippersnapper_servo_ServoAdd *msg_add = _servo_model->GetServoAddMsg();
  uint8_t pin = atoi(msg_add->servo_pin + 1);
  _servo_hardware[_active_servo_pins] = new ServoHardware(
      pin, (int)msg_add->min_pulse_width, (int)msg_add->max_pulse_width,
      (int)msg_add->servo_freq);
  // Attempt to attach the servo to the pin
  bool did_attach = false;
  did_attach = _servo_hardware[_active_servo_pins]->ServoAttach();

  // Write the default minimum to a servo
  if (did_attach) {
    _servo_hardware[_active_servo_pins]->ServoWrite(MIN_SERVO_PULSE_WIDTH);
    WS_DEBUG_PRINT("[servo] Servo attached to pin: ");
    WS_DEBUG_PRINTLN(msg_add->servo_pin);
    _active_servo_pins++;
  } else {
    WS_DEBUG_PRINTLN("[servo] Error: Failed to attach servo to pin!");
    delete _servo_hardware[_active_servo_pins];
    _servo_hardware[_active_servo_pins] = nullptr;
  }

  // Publish ServoAdded message to IO
  _servo_model->EncodeServoAdded(msg_add->servo_pin, did_attach);
  if (!WsV2.PublishSignal(wippersnapper_signal_DeviceToBroker_servo_added_tag,
                          _servo_model->GetServoAddedMsg())) {
    WS_DEBUG_PRINTLN("[servo] Error: Failed publishing a ServoAdded message!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief  Handles a ServoWrite message
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool ServoController::Handle_Servo_Write(pb_istream_t *stream) {
  if (!_servo_model->DecodeServoWrite(stream)) {
    WS_DEBUG_PRINTLN("[servo] Error: Failed to decode ServoWrite message!");
    return false;
  }
  wippersnapper_servo_ServoWrite *msg_write = _servo_model->GetServoWriteMsg();
  uint8_t pin = atoi(msg_write->servo_pin + 1);
  int servo_idx = GetServoIndex(pin);
  if (servo_idx == -1) {
    WS_DEBUG_PRINTLN("[servo] Error: Servo pin not found!");
    return false;
  }
  // Write the pulse width to the servo
  _servo_hardware[servo_idx]->ServoWrite(msg_write->pulse_width);
  WS_DEBUG_PRINT("[servo] Set Pulse Width: ");
  WS_DEBUG_PRINT(msg_write->pulse_width);
  WS_DEBUG_PRINT(" µs on pin: ");
  WS_DEBUG_PRINT(msg_write->servo_pin);
  return true;
}

/**************************************************************************/
/*!
    @brief  Handles a ServoRemove message
    @param  stream
            pb_istream_t to decode from
    @returns True if successful, False otherwise
*/
/**************************************************************************/
bool ServoController::Handle_Servo_Remove(pb_istream_t *stream) {
  if (!_servo_model->DecodeServoRemove(stream)) {
    WS_DEBUG_PRINTLN("[servo] Error: Failed to decode ServoRemove message!");
    return false;
  }
  wippersnapper_servo_ServoRemove *msg_remove =
      _servo_model->GetServoRemoveMsg();
  uint8_t pin = atoi(msg_remove->servo_pin + 1);
  int servo_idx = GetServoIndex(pin);
  if (servo_idx == -1) {
    WS_DEBUG_PRINTLN("[servo] Error: Servo pin not found!");
    return false;
  }

  if (_active_servo_pins <= 0) {
    WS_DEBUG_PRINTLN("[servo] Error: No active servos!");
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
  WS_DEBUG_PRINTLN(msg_remove->servo_pin);
  return true;
}

int ServoController::GetServoIndex(uint8_t pin) {
  for (int i = 0; i < _active_servo_pins; i++) {
    if (_servo_hardware[i]->GetPin() == pin) {
      return i;
    }
  }
  return -1;
}