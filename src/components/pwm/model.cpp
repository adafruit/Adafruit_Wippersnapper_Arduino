/*!
 * @file src/components/pwm/model.cpp
 *
 * Model for the pwm.proto message.
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
#include "model.h"

PWMModel::PWMModel() {}

PWMModel::~PWMModel() {}

bool PWMModel::DecodePWMAdd(pb_istream_t *stream) {
  return false;
}

wippersnapper_pwm_PWMAdd *PWMModel::GetPWMAddMsg() {
}

bool PWMModel::EncodePWMAdded(char *pin_name, bool did_attach) {
  return false;
}

wippersnapper_pwm_PWMAdded *PWMModel::GetPWMAddedMsg() {
}

bool PWMModel::DecodePWMRemove(pb_istream_t *stream) {
  return false;
}

wippersnapper_pwm_PWMRemove *PWMModel::GetPWMRemoveMsg() {
}

bool PWMModel::DecodePWMWriteDutyCycle(pb_istream_t *stream) {
  return false;
}

wippersnapper_pwm_PWMWriteDutyCycle *PWMModel::GetPWMWriteDutyCycleMsg() {
}

bool PWMModel::DecodePWMWriteDutyCycleMulti(pb_istream_t *stream) {
  return false;
}

wippersnapper_pwm_PWMWriteDutyCycleMulti *PWMModel::GetPWMWriteDutyCycleMultiMsg() {
}

bool PWMModel::DecodePWMWriteFrequency(pb_istream_t *stream) {
  return false;
}

wippersnapper_pwm_PWMWriteFrequency *PWMModel::GetPWMWriteFrequencyMsg() {
}
