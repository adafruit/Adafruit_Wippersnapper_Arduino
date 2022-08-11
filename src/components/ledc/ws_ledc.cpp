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
 * @section author Author
 *
 * Written by Brent Rubell for Adafruit Industries, 2022.
 *
 * @section license License
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#if defined(ARDUINO_FEATHER_ESP32)

#include "ws_ledc.h"

WipperSnapper_Component_LEDC::WipperSnapper_Component_LEDC() {
  // reset active pins
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    _ledcPins[i].isActive = false;
  }
}

WipperSnapper_Component_LEDC::~WipperSnapper_Component_LEDC() {
  // detach all active pins and de-allocate them
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    detachPin(_ledcPins[i].pin);
    _ledcPins[i].isActive = false;
  }
}

/**************************************************************************/
/*!
    @brief  Allocates a timer + channel for a pin and attaches it.
    @param  pin  Desired GPIO pin number.
    @param  freq Desired timer frequency, in Hz.
    @return The channel number if the pin was successfully attached,
            otherwise 255.
*/
/**************************************************************************/
uint8_t WipperSnapper_Component_LEDC::attachPin(uint8_t pin, double freq) {
  // have we already attached this pin?
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    if (_ledcPins[i].pin == pin)
      return 255;
  }

  // allocate chanel
  uint8_t chanNum = _allocateChannel(freq);
  if (chanNum == 255)
    return chanNum;

  // attach pin to channel
  ledcAttachPin(pin, chanNum);

  // allocate pin in pool
  _ledcPins[chanNum].pin = pin;

  return chanNum;
}

/**************************************************************************/
/*!
    @brief  Detaches a pin and de-allocates it from the manager.
    @param  pin  Desired GPIO pin number.
*/
/**************************************************************************/
void WipperSnapper_Component_LEDC::detachPin(uint8_t pin) {
  // find the channel corresponding to the pin
  uint8_t chan;
  for (int i = 0; i < sizeof(_ledcPins); i++) {
    if (_ledcPins[i].pin == pin) {
      chan = _ledcPins[i].chan;
      break;
    }
  }

  // detach the pin
  ledcDetachPin(pin);

  // de-allocate the pin and the channel
  for (int i = 0; i < sizeof(_ledcPins); i++) {
    if (_ledcPins[i].pin == pin) {
      _ledcPins[i].pin = 0;
      _ledcPins[i].chan = 255;
      _ledcPins[i].isActive = false;
      break;
    }
  }
}

/**************************************************************************/
/*!
    @brief  Allocates a channel and timer.
    @param  freq Desired timer frequency, in Hz.
    @return The channel number if the timer was successfully initialized,
            otherwise 255.
*/
/**************************************************************************/
uint8_t WipperSnapper_Component_LEDC::_allocateChannel(double freq) {
  // attempt to allocate an inactive channel
  uint8_t chanNum = 255;
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    if (_ledcPins[i].isActive == false) {
      chanNum = i;
      break;
    }
  }

  // did we fail to allocate?
  if (chanNum == 255)
    return 255;

  // attempt to set up a ledc_timer on the free channel
  double rc = ledcSetup(uint8_t(chanNum), freq, 16);
  if (rc == 0)
    return 255;

  // Assign
  _ledcPins[chanNum].chan = chanNum;
  _ledcPins[chanNum].isActive = true;

  return chanNum;
}

/**************************************************************************/
/*!
    @brief  Sets the duty cycle of a pin
    @param  pin  Desired GPIO pin to write to.
    @param  freq Desired duty cycle.
*/
/**************************************************************************/
void WipperSnapper_Component_LEDC::setDuty(uint8_t pin, uint32_t duty) {
  // find the channel corresponding to the pin
  uint8_t chan;
  for (int i = 0; i < sizeof(_ledcPins); i++) {
    if (_ledcPins[i].pin == pin) {
      chan = _ledcPins[i].chan;
      break;
    }
  }
  // set the channel's duty cycle
  // debug...
  // Serial.print("Writing duty cycle: ");
  // Serial.print(duty);
  // Serial.print("(%) to channel#: ");
  // Serial.println(chan);
  ledcWrite(chan, duty);
}

#endif //ARDUINO_ARCH_ESP32