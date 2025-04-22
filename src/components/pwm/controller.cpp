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

/**************************************************************************/
/*!
    @brief  Ctor for PWMController.
*/
/**************************************************************************/
PWMController::PWMController() {
  _pwm_model = new PWMModel();
  _active_pwm_pins = 0;
}

/**************************************************************************/
/*!
    @brief  Dtor for PWMController.
*/
/**************************************************************************/
PWMController::~PWMController() { delete _pwm_model; }

/**************************************************************************/
/*!
    @brief  Handles the PWM_Add message.
    @param  stream The stream containing the message data.
    @return True if the message was handled successfully, false otherwise.
*/
/**************************************************************************/
bool PWMController::Handle_PWM_Add(pb_istream_t *stream) {
  bool did_attach;
  if (!_pwm_model->DecodePWMAdd(stream)) {
    WS_DEBUG_PRINTLN("[pwm] Failed to decode PWMAdd message!");
    return false;
  }
  wippersnapper_pwm_PWMAdd msg_add = *_pwm_model->GetPWMAddMsg();
  uint8_t pin = atoi(msg_add.pin + 1);
  _pwm_hardware[_active_pwm_pins] = new PWMHardware();

  WS_DEBUG_PRINT("[pwm] Attaching pin: ");
  WS_DEBUG_PRINT(msg_add.pin);
  did_attach = _pwm_hardware[_active_pwm_pins]->AttachPin(
      pin, (uint32_t)msg_add.frequency, (uint32_t)msg_add.resolution);
  if (! did_attach) {
    WS_DEBUG_PRINTLN("[pwm] Failed to attach pin!");
    delete _pwm_hardware[_active_pwm_pins];
  } else {
    _active_pwm_pins++;
  }

  // Publish PixelsAdded message to the broker
  if (!_pwm_model->EncodePWMAdded(msg_add.pin, did_attach)) {
    WS_DEBUG_PRINTLN("[pwm]: Failed to encode PWMAdded message!");
    return false;
  }
  if (!WsV2.PublishSignal(wippersnapper_signal_DeviceToBroker_pwm_added_tag,
                          _pwm_model->GetPWMAddedMsg())) {
    WS_DEBUG_PRINTLN("[PWM]: Unable to publish PWMAdded message!");
    return false;
  }
  WS_DEBUG_PRINTLN("...attached!");
  return true;
}

/**************************************************************************/
/*!
    @brief  Handles the PWM_Remove message.
    @param  stream The stream containing the message data.
    @return True if the message was handled successfully, false otherwise.
*/
/**************************************************************************/
bool PWMController::Handle_PWM_Remove(pb_istream_t *stream) {
  if (!_pwm_model->DecodePWMRemove(stream)) {
    WS_DEBUG_PRINTLN("[pwm] Error: Failed to decode PWMRemove message!");
    return false;
  }
  wippersnapper_pwm_PWMRemove msg_remove = *_pwm_model->GetPWMRemoveMsg();
  uint8_t pin = atoi(msg_remove.pin + 1);
  int pin_idx = GetPWMHardwareIdx(pin);
  if (pin_idx == -1) {
    WS_DEBUG_PRINTLN("[pwm] Error: pin not found!");
    return false;
  }

  // Detach and free the pin for other uses
  WS_DEBUG_PRINT("[pwm] Detaching pin: ");
  WS_DEBUG_PRINT(msg_remove.pin);
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

/**************************************************************************/
/*!
    @brief  Returns the index of the PWM hardware object that corresponds
            to the given pin.
    @param  pin The pin number to search for.
    @return The index of the PWM hardware object, or -1 if not found.
*/
/**************************************************************************/
int PWMController::GetPWMHardwareIdx(uint8_t pin) {
  for (int i = 0; i < _active_pwm_pins; i++) {
    if (_pwm_hardware[i]->GetPin() == pin) {
      return i;
    }
  }
  return -1;
}

/**************************************************************************/
/*!
    @brief  Handles the PWM_Write_DutyCycle message.
    @param  stream The stream containing the message data.
    @return True if the message was handled successfully, false otherwise.
*/
/**************************************************************************/
bool PWMController::Handle_PWM_Write_DutyCycle(pb_istream_t *stream) {
  if (!_pwm_model->DecodePWMWriteDutyCycle(stream)) {
    WS_DEBUG_PRINTLN(
        "[pwm] Error: Failed to decode PWMWriteDutyCycle message!");
    return false;
  }

  wippersnapper_pwm_PWMWriteDutyCycle msg_write_duty_cycle =
      *_pwm_model->GetPWMWriteDutyCycleMsg();
  uint8_t pin = atoi(msg_write_duty_cycle.pin + 1);

  // Check if the pin is already attached
  int pin_idx = GetPWMHardwareIdx(pin);
  if (pin_idx == -1) {
    WS_DEBUG_PRINTLN("[pwm] Error: pin not found!");
    return false;
  }

  // Write the duty cycle to the pin
  if (!_pwm_hardware[pin_idx]->WriteDutyCycle(
          msg_write_duty_cycle.duty_cycle)) {
    WS_DEBUG_PRINTLN("[pwm] Error: Failed to write duty cycle!");
    return false;
  }
  WS_DEBUG_PRINTLN("[pwm] Wrote duty cycle: ");
  WS_DEBUG_PRINT(msg_write_duty_cycle.duty_cycle);
  WS_DEBUG_PRINTLN(" to pin: ");
  WS_DEBUG_PRINT(msg_write_duty_cycle.pin);
  return true;
}

/**************************************************************************/
/*!
    @brief  Handles the PWM_Write_DutyCycle_Multi message.
    @param  stream The stream containing the message data.
    @return True if the message was handled successfully, false otherwise.
*/
/**************************************************************************/
bool PWMController::Handle_PWM_Write_DutyCycle_Multi(pb_istream_t *stream) {
  return false;
}

/**************************************************************************/
/*!
    @brief  Handles the PWM_Write_Frequency message.
    @param  stream The stream containing the message data.
    @return True if the message was handled successfully, false otherwise.
*/
/**************************************************************************/
bool PWMController::Handle_PWM_Write_Frequency(pb_istream_t *stream) {
  if (!_pwm_model->DecodePWMWriteFrequency(stream)) {
    WS_DEBUG_PRINTLN(
        "[pwm] Error: Failed to decode PWMWriteFrequency message!");
    return false;
  }
  wippersnapper_pwm_PWMWriteFrequency msg_write_frequency =
      *_pwm_model->GetPWMWriteFrequencyMsg();
  uint8_t pin = atoi(msg_write_frequency.pin + 1);
  // Check if the pin is already attached
  int pin_idx = GetPWMHardwareIdx(pin);
  if (pin_idx == -1) {
    WS_DEBUG_PRINTLN("[pwm] Error: pin not found!");
    return false;
  }

  if (!_pwm_hardware[pin_idx]->WriteTone(msg_write_frequency.frequency) ==
      msg_write_frequency.frequency) {
    WS_DEBUG_PRINTLN("[pwm] Error: Failed to write frequency!");
    return false;
  }
  WS_DEBUG_PRINTLN("[pwm] Wrote frequency: ");
  WS_DEBUG_PRINT(msg_write_frequency.frequency);
  WS_DEBUG_PRINTLN(" to pin: ");
  WS_DEBUG_PRINT(msg_write_frequency.pin);
  return true;
}