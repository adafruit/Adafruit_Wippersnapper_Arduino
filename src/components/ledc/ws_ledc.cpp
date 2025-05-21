/*!
 * @file ws_ledc.cpp
 *
 * This is the documentation for WipperSnapper's LEDC peripheral
 * management API. It is used by drivers like ledc_servo.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries, 2022-2023
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#if defined(ARDUINO_ARCH_ESP32)

#include "ws_ledc.h"

/**************************************************************************/
/*!
    @brief  Sets up a LEDC pin with given frequency and resolution.
    @param  pin  Desired GPIO pin number.
    @param  freq Desired timer frequency, in Hz.
    @param  resolution Desired timer resolution, in bits.
    @return True if configuration is successful. False is returned if error
   occurs and LEDC channel was not configured.
*/
/**************************************************************************/
bool ws_ledc::attachPin(uint8_t pin, uint32_t freq, uint8_t resolution) {
  return ledcAttach(pin, freq, resolution);
}

/**************************************************************************/
/*!
    @brief  Detaches a pin from LEDC.
    @param  pin  Desired GPIO pin number.
    @return True if successfully detached, False otherwise.
*/
/**************************************************************************/
bool ws_ledc::detachPin(uint8_t pin) { return ledcDetach(pin); }

/**************************************************************************/
/*!
    @brief  Arduino AnalogWrite function, but for ESP32's LEDC.
    @param  pin  The desired pin to write to.
    @param  value The duty cycle.
    @return True if PWM value written to LEDC pin, False otherwise.
*/
/**************************************************************************/
bool ws_ledc::analogWrite(uint8_t pin, int value) {
  if (value > 255 || value < 0)
    return false;

  // Calculate duty cycle for the `value` passed in
  // (assumes 12-bit resolution, 2^12)
  //uint32_t dutyCycle = (4095 / 255) * min(value, 255);
  uint32_t dutyCycle =(uint32_t)(((double)4095 / 255.0) * min((uint32_t)value, (uint32_t)255));

  // Call duty cycle write
  return setDuty(pin, dutyCycle);
}

/**************************************************************************/
/*!
    @brief  Sets the duty cycle of a LEDC pin.
    @param  pin  Desired GPIO pin to write to.
    @param  duty  Desired duty cycle.
    @return True if duty cycle was set, False otherwise.
*/
/**************************************************************************/
bool ws_ledc::setDuty(uint8_t pin, uint32_t duty) {
  return ledcWrite(pin, duty);
}

/**************************************************************************/
/*!
    @brief  Writes a square wave with a fixed duty cycle and variable
            frequency to a pin. Used by piezo buzzers and speakers.
    @param  pin  The desired pin to write to.
    @param  freq The frequency of the tone, in Hz.
    @return The frequency of the LEDC pin. 0 if error occurs and LEDC pin was
   not configured.
*/
/**************************************************************************/
uint32_t ws_ledc::tone(uint8_t pin, uint32_t freq) {
  return ledcWriteTone(pin, freq);
}

#endif // ARDUINO_ARCH_ESP32