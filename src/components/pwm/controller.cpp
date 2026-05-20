/*!
 * @file src/components/pwm/controller.cpp
 *
 * Controller for the pwm API
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
    @brief  Ctor for PWMController.
*/
PWMController::PWMController() {
  _pwm_model = new PWMModel();
  _active_pwm_pins = 0;
}

/*!
    @brief  Dtor for PWMController.
*/
PWMController::~PWMController() { delete _pwm_model; }

/*!
    @brief  Routes messages using the pwm.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool PWMController::Router(pb_istream_t *stream) {
  // Attempt to decode the PWM B2D envelope
  ws_pwm_B2D b2d = ws_pwm_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_pwm_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN("[pwm] ERROR: Unable to decode PWM B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_pwm_B2D_add_tag:
    res = Handle_PWM_Add(&b2d.payload.add);
    break;
  case ws_pwm_B2D_remove_tag:
    res = Handle_PWM_Remove(&b2d.payload.remove);
    break;
  case ws_pwm_B2D_write_tag:
    res = Handle_PWM_Write(&b2d.payload.write);
    break;
  default:
    WS_DEBUG_PRINTLN("[pwm] WARNING: Unsupported PWM payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles the PWM_Add message.
    @param  msg The PWMAdd message.
    @return True if the message was handled successfully, false otherwise.
*/
bool PWMController::Handle_PWM_Add(ws_pwm_Add *msg) {
  uint8_t pin = atoi(msg->pin + 1);
  // Create new PWM hardware object and attempt to attach the pin
  _pwm_hardware[_active_pwm_pins] = new PWMHardware();
  if (!_pwm_hardware[_active_pwm_pins]->AttachPin(pin, (uint32_t)msg->frequency,
                                                  (uint32_t)msg->resolution)) {
    Ws.error_handler->publishComponentError(msg->pin,
                                               "Failed to attach pin");
    delete _pwm_hardware[_active_pwm_pins];
    return false;
  }

  WS_DEBUG_PRINT("[pwm] Attached pin: ");
  WS_DEBUG_PRINTVAR(msg->pin);
  _active_pwm_pins++;

  return true;
}

/*!
    @brief  Handles the PWM_Remove message.
    @param  msg The PWMRemove message.
    @return True if the message was handled successfully, false otherwise.
*/
bool PWMController::Handle_PWM_Remove(ws_pwm_Remove *msg) {
  uint8_t pin = atoi(msg->pin + 1);
  int pin_idx = GetPWMHardwareIdx(pin);
  if (pin_idx == -1) {
    Ws.error_handler->publishComponentError(msg->pin, "Failed to find pin");
    return false;
  }

  // Attempt to detach and free pin
  WS_DEBUG_PRINT("[pwm] Detaching pin: ");
  WS_DEBUG_PRINTVAR(msg->pin);
  if (!_pwm_hardware[pin_idx]->DetachPin()) {
    Ws.error_handler->publishComponentError(msg->pin,
                                               "Failed to detach pin");
    return false;
  }
  delete _pwm_hardware[pin_idx];
  _pwm_hardware[pin_idx] = nullptr;

  // Reorganize _active_pwm_pins
  _active_pwm_pins--;
  for (int i = pin_idx; i < _active_pwm_pins; i++) {
    _pwm_hardware[i] = _pwm_hardware[i + 1];
  }
  _pwm_hardware[_active_pwm_pins] = nullptr;

  return true;
}

/*!
    @brief  Returns the index of the PWM hardware object that corresponds
            to the given pin.
    @param  pin The pin number to search for.
    @return The index of the PWM hardware object, or -1 if not found.
*/
int PWMController::GetPWMHardwareIdx(uint8_t pin) {
  for (int i = 0; i < _active_pwm_pins; i++) {
    if (_pwm_hardware[i]->GetPin() == pin) {
      return i;
    }
  }
  return -1;
}

/*!
    @brief  Handles the PWM_Write message.
    @param  msg The PWMWrite message.
    @return True if the message was handled successfully, false otherwise.
*/
bool PWMController::Handle_PWM_Write(ws_pwm_Write *msg) {
  uint8_t pin = atoi(msg->pin + 1);
  int pin_idx = GetPWMHardwareIdx(pin);
  if (pin_idx == -1) {
    Ws.error_handler->publishComponentError(msg->pin, "Failed to find pin");
    return false;
  }

  // Check which payload type we have
  if (msg->which_payload == ws_pwm_Write_duty_cycle_tag) {
    // Attempt to write the duty cycle to the pin
    if (!_pwm_hardware[pin_idx]->WriteDutyCycle(msg->payload.duty_cycle)) {
      Ws.error_handler->publishComponentError(msg->pin, "Failed to write duty cycle");
      return false;
    }

    WS_DEBUG_PRINT("[pwm] Wrote duty cycle: ");
    WS_DEBUG_PRINTVAR(msg->payload.duty_cycle);
    WS_DEBUG_PRINT(" to pin: ");
    WS_DEBUG_PRINTLNVAR(msg->pin);
  } else if (msg->which_payload == ws_pwm_Write_frequency_tag) {
    // Attempt to write the frequency to the pin
    if (_pwm_hardware[pin_idx]->WriteTone(msg->payload.frequency) !=
        msg->payload.frequency) {
      Ws.error_handler->publishComponentError(msg->pin, "Failed to write frequency");
      return false;
    }
    WS_DEBUG_PRINT("[pwm] Wrote frequency: ");
    WS_DEBUG_PRINTVAR(msg->payload.frequency);
    WS_DEBUG_PRINT(" to pin: ");
    WS_DEBUG_PRINTLNVAR(msg->pin);
  } else {
    Ws.error_handler->publishComponentError(msg->pin, "Invalid payload type");
    return false;
  }

  return true;
}