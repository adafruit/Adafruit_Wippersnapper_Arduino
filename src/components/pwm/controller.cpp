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
  _pwm_hardware = new PWMHardware();
}

PWMController::~PWMController() {
  delete _pwm_model;
  delete _pwm_hardware;
}

bool PWMController::Handle_PWM_Add(pb_istream_t *stream) { return false; }

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
