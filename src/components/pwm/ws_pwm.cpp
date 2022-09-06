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
*/
/**************************************************************************/
ws_pwm::ws_pwm() {
    // TODO
}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_pwm::~ws_pwm() {
    // TODO
}

bool ws_pwm::attach(uint8_t pin, double freq, uint8_t resolution) {
    // TODO
    return true;
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