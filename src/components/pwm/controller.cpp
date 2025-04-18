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

PWMController::PWMController() {
  _pwm_model = new PWMModel();
  _active_pwm_pins = 0;
}

PWMController::~PWMController() { delete _pwm_model; }

bool PWMController::Handle_PWM_Add(pb_istream_t *stream) {
  bool did_attach;
  if (!_pwm_model->DecodePWMAdd(stream)) {
    WS_DEBUG_PRINTLN("[pwm] Failed to decode PWMAdd message!");
    return false;
  }
  wippersnapper_pwm_PWMAdd msg_add = *_pwm_model->GetPWMAddMsg();
  uint8_t pin = atoi(msg_add.pin + 1);
  _pwm_hardware[_active_pwm_pins] = new PWMHardware();

  did_attach = _pwm_hardware[_active_pwm_pins]->AttachPin(
      pin, (uint32_t)msg_add.frequency, (uint32_t)msg_add.resolution);
  if (!did_attach) {
    WS_DEBUG_PRINTLN("[pwm] Failed to attach pin!");
    // TODO: Test init. failure to see if the below line crashes?
    // TODO: if it doesn't, we should probably implement this
    // delete in the pixel controller class to avoid memory leaks
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
  return true;
}

bool PWMController::Handle_PWM_Write_DutyCycle(pb_istream_t *stream) {
  return false;
}

bool PWMController::Handle_PWM_Write_DutyCycle_Multi(pb_istream_t *stream) {
  return false;
}

bool PWMController::Handle_PWM_Write_Frequency(pb_istream_t *stream) {
  return false;
}

bool PWMController::Handle_PWM_Remove(pb_istream_t *stream) { return false; }
