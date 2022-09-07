/*!
 * @file ws_pwm.cpp
 *
 * PWM component for WipperSnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * Written by Brent Rubell for Adafruit Industries, 2022.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#include "ws_pwm.h"

/**************************************************************************/
/*!
    @brief  Constructor
    @param  ledcManager  Pointer to LEDC driver.
*/
/**************************************************************************/
ws_pwm::ws_pwm(ws_ledc *ledcManager) { _ledcMgr = ledcManager; }

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_pwm::~ws_pwm() {
  // TODO
}

bool ws_pwm::attach(uint8_t pin, double freq, uint8_t resolution) {
  bool is_attached = true;
#ifndef ARDUINO_ARCH_ESP32
  return true; // mock on non-ledc
#endif
  // uint8_t rc = _ledcMgr->attachPin(pin, freq, resolution);
  // uint8_t rc = ledcMgr.attachPin(pin, freq, resolution);
  // uint8_t rc = WS.
  // if (rc == LEDC_CH_ERR)
  //    is_attached = false;
  return is_attached;
}

void ws_pwm::detach(uint8_t pin) {
  // TODO
}

void ws_pwm::writeDutyCycle(uint8_t pin, int dutyCycle) {
  // TODO
}

void ws_pwm::writeTone(uint8_t pin, uint32_t freq) {
  // TODO
}

void ws_pwm::noTone(uint8_t pin) {
  // TODO
}