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
  bool did_attach;
  uint8_t pin = atoi(msg->pin + 1);
  _pwm_hardware[_active_pwm_pins] = new PWMHardware();

  WS_DEBUG_PRINT("[pwm] Attaching pin: ");
  WS_DEBUG_PRINT(msg->pin);
  did_attach = _pwm_hardware[_active_pwm_pins]->AttachPin(
      pin, (uint32_t)msg->frequency, (uint32_t)msg->resolution);
  if (!did_attach) {
    WS_DEBUG_PRINTLN("[pwm] Failed to attach pin!");
    delete _pwm_hardware[_active_pwm_pins];
  } else {
    _active_pwm_pins++;
  }

  // Publish PixelsAdded message to the broker
  if (!_pwm_model->EncodePWMAdded(msg->pin, did_attach)) {
    WS_DEBUG_PRINTLN("[pwm]: Failed to encode PWMAdded message!");
    return false;
  }
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_pwm_tag,
                     _pwm_model->GetPWMAddedMsg())) {
    WS_DEBUG_PRINTLN("[PWM]: Unable to publish PWMAdded message!");
    return false;
  }
  WS_DEBUG_PRINTLN("...attached!");
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
    WS_DEBUG_PRINTLN("[pwm] Error: pin not found!");
    return false;
  }

  // Detach and free the pin for other uses
  WS_DEBUG_PRINT("[pwm] Detaching pin: ");
  WS_DEBUG_PRINT(msg->pin);
  if (_pwm_hardware[pin_idx] != nullptr) {
    bool detach_result = _pwm_hardware[pin_idx]->DetachPin();
    if (!detach_result) {
      WS_DEBUG_PRINTLN("[pwm] Error: Failed to detach pin.");
    }
    delete _pwm_hardware[pin_idx];
    _pwm_hardware[pin_idx] = nullptr;
  } else {
    WS_DEBUG_PRINTLN("[pwm] Error: Pin not attached!");
  }

  // Reorganize _active_pwm_pins
  _active_pwm_pins--;
  for (int i = pin_idx; i < _active_pwm_pins; i++) {
    _pwm_hardware[i] = _pwm_hardware[i + 1];
  }
  _pwm_hardware[_active_pwm_pins] = nullptr;
  WS_DEBUG_PRINTLN("...detached!");
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
    WS_DEBUG_PRINTLN("[pwm] Error: pin not found!");
    return false;
  }

  // Check which payload type we have
  if (msg->which_payload == ws_pwm_Write_duty_cycle_tag) {
    // Write the duty cycle to the pin
    if (!_pwm_hardware[pin_idx]->WriteDutyCycle(msg->payload.duty_cycle)) {
      WS_DEBUG_PRINTLN("[pwm] Error: Failed to write duty cycle!");
      return false;
    }
    WS_DEBUG_PRINTLN("[pwm] Wrote duty cycle: ");
    WS_DEBUG_PRINT(msg->payload.duty_cycle);
    WS_DEBUG_PRINTLN(" to pin: ");
    WS_DEBUG_PRINT(msg->pin);
  } else if (msg->which_payload == ws_pwm_Write_frequency_tag) {
    // Write the frequency to the pin
    if (_pwm_hardware[pin_idx]->WriteTone(msg->payload.frequency) !=
        msg->payload.frequency) {
      WS_DEBUG_PRINTLN("[pwm] Error: Failed to write frequency!");
      return false;
    }
    WS_DEBUG_PRINTLN("[pwm] Wrote frequency: ");
    WS_DEBUG_PRINT(msg->payload.frequency);
    WS_DEBUG_PRINTLN(" to pin: ");
    WS_DEBUG_PRINT(msg->pin);
  } else {
    WS_DEBUG_PRINTLN("[pwm] Error: Invalid payload type!");
    return false;
  }

  return true;
}