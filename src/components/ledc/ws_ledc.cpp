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
 * Written by Brent Rubell for Adafruit Industries, 2022.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#if defined(ARDUINO_ARCH_ESP32)

#include "ws_ledc.h"

/**************************************************************************/
/*!
    @brief  Constructor
*/
/**************************************************************************/
ws_ledc::ws_ledc() {
  // reset active pins
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    _ledcPins[i].isActive = false;
  }
}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_ledc::~ws_ledc() {
  // detach all active pins and de-allocate them
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    detachPin(_ledcPins[i].pin);
    _ledcPins[i].isActive = false;
  }
}

/**************************************************************************/
/*!
    @brief  Checks if a channel has already been allocated for a pin.
    @param  pin  Desired GPIO pin number.
    @return The channel number if the pin was successfully or already
            attached, otherwise LEDC_CH_ERR.
*/
/**************************************************************************/
uint8_t ws_ledc::getChannel(uint8_t pin) {
  // have we already attached this pin?
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    if (_ledcPins[i].pin == pin)
      return _ledcPins[i].chan;
  }
  return LEDC_CH_ERR;
}

/**************************************************************************/
/*!
    @brief  Allocates a timer + channel for a pin and attaches it.
    @param  pin  Desired GPIO pin number.
    @param  freq Desired timer frequency, in Hz.
    @param  resolution Desired timer resolution, in bits.
    @return The channel number if the pin was successfully or already
            attached, otherwise 255.
*/
/**************************************************************************/
uint8_t ws_ledc::attachPin(uint8_t pin, double freq, uint8_t resolution) {
  // have we already attached this pin?
  for (int i = 0; i < MAX_LEDC_PWMS; i++) {
    if (_ledcPins[i].pin == pin)
      return _ledcPins[i].chan;
  }

  // allocate chanel
  uint8_t chanNum = allocateChannel(freq, resolution);
  if (chanNum == LEDC_CH_ERR)
    return chanNum;

  // attach pin to channel
  ledcAttachPin(pin, chanNum);

  // allocate pin in pool
  _ledcPins[chanNum].pin = pin;
  _ledcPins[chanNum].chan = chanNum;

  return chanNum;
}

/**************************************************************************/
/*!
    @brief  Detaches a pin and de-allocates it from the manager.
    @param  pin  Desired GPIO pin number.
*/
/**************************************************************************/
void ws_ledc::detachPin(uint8_t pin) {
  // de-allocate the pin and the channel
  for (int i = 0; i < sizeof(_ledcPins); i++) {
    if (_ledcPins[i].pin == pin) {
      _ledcPins[i].pin = 0;
      _ledcPins[i].chan = 255;
      _ledcPins[i].isActive = false;
      break;
    }
  }

  // detach the pin
  ledcDetachPin(pin);
}

/**************************************************************************/
/*!
    @brief  Allocates a channel and timer.
    @param  freq       Timer frequency, in Hz.
    @param  resolution Timer resolution, in bits.
    @return The channel number if the timer was successfully initialized,
            otherwise 255.
*/
/**************************************************************************/
uint8_t ws_ledc::allocateChannel(double freq, uint8_t resolution = 16) {
  // obtain an inactive channel number
  uint8_t chanNum = getInactiveChannel();
  if (chanNum == LEDC_CH_ERR)
    return LEDC_CH_ERR; // failed to obtain inactive channel #

  // attempt to set up a ledc_timer on the free channel
  double rc = ledcSetup(uint8_t(chanNum), freq, resolution);
  if (rc == 0)
    return 255;

  // Assign
  _ledcPins[chanNum].chan = chanNum;
  _ledcPins[chanNum].isActive = true;

  return chanNum;
}

/**************************************************************************/
/*!
    @brief    Returns an inactive LEDC channel number.
    @returns  Inactive channel number if free, otherwise LEDC_CH_ERR.
*/
/**************************************************************************/
uint8_t ws_ledc::getInactiveChannel() {
  for (int ch = 0; ch < sizeof(_ledcPins); ch++) {
    if (_ledcPins[ch].isActive == false) {
      return ch;
    }
  }
  return LEDC_CH_ERR;
}

/**************************************************************************/
/*!
    @brief  Arduino AnalogWrite function, but for ESP32's LEDC.
    @param  pin  The desired pin to write to.
    @param  value The duty cycle.
*/
/**************************************************************************/
void ws_ledc::analogWrite(uint8_t pin, int value) {
  if (value > 255 || value < 0)
    return;

  uint8_t ch;
  ch = getChannel(pin);
  if (ch == LEDC_CH_ERR) {
    Serial.println("ERROR: Pin not attached to channel");
    return;
  }

  // perform duty cycle calculation provided value
  // (assumes 12-bit resolution, 2^12)
  uint32_t dutyCycle = (4095 / 255) * min(value, 255);

  // set the duty cycle of the pin
  setDuty(pin, dutyCycle);
}

/**************************************************************************/
/*!
    @brief  Sets the duty cycle of a pin
    @param  pin  Desired GPIO pin to write to.
    @param  duty  Desired duty cycle.
*/
/**************************************************************************/
void ws_ledc::setDuty(uint8_t pin, uint32_t duty) {
  // find the channel corresponding to the pin
  uint8_t chan = 0;
  for (int i = 0; i < sizeof(_ledcPins); i++) {
    if (_ledcPins[i].pin == pin) {
      chan = _ledcPins[i].chan;
      break;
    }
  }

  // set the channel's duty cycle
  ledcWrite(chan, duty);
}

/**************************************************************************/
/*!
    @brief  Writes a square wave with a fixed duty cycle and variable
            frequency to a pin. Used by piezo buzzers and speakers.
    @param  pin  The desired pin to write to.
    @param  freq The frequency of the tone, in Hz.
*/
/**************************************************************************/
void ws_ledc::tone(uint8_t pin, uint32_t freq) {
  uint8_t ch;
  ch = getChannel(pin);
  if (ch == LEDC_CH_ERR) {
    // Serial.println("ERROR: Pin not previously attached!");
  }
  ledcWriteTone(ch, freq);
}

#endif // ARDUINO_ARCH_ESP32