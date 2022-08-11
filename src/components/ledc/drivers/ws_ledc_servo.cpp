/*!
 * @file ws_ledc_servo.cpp
 *
 * Driver for ESP32 servo control using the WipperSnapper
 * LEDC peripheral manager API.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Brent Rubell for Adafruit Industries, 2022.
 *
 * writeMicroseconds calculation from ESP32Servo
 * https://github.com/madhephaestus/ESP32Servo/blob/master/src/ESP32Servo.cpp
 * Copyright (c) 2017 John K. Bennett.
 *
 * @section license License
 *
 * GNU Lesser General Public License, all text here must be included in any
 * redistribution.
 *
 */
#if defined(ARDUINO_FEATHER_ESP32)
#include "ws_ledc_servo.h"

/**************************************************************************/
/*!
    @brief  Constructor.
*/
/**************************************************************************/
ws_ledc_servo::ws_ledc_servo() {}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_ledc_servo::~ws_ledc_servo() {
  detach(); // detach the active pin
}

void ws_ledc_servo::setLEDCDriver(WipperSnapper_Component_LEDC *ledcManager) {
  _ledcMgr = ledcManager;
}

/**************************************************************************/
/*!
    @brief    Attaches a servo object to a pin.
    @param    pin  Desired GPIO pin.
    @param    minPulseWidth  Minimum pulsewidth, in uS.
    @param    maxPulseWidth  Maximum pulsewidth, in uS.
    @returns  Channel number if a servo is successfully attached to a pin,
              otherwise 255.
*/
/**************************************************************************/
uint8_t ws_ledc_servo::attach(int pin, int minPulseWidth, int maxPulseWidth,
                              int servoFreq) {
  // Attempt to attach a pin to ledc channel
  uint8_t chan = _ledcMgr->attachPin((uint8_t)pin, (double)servoFreq);
  if (chan == 255) // error!
    return chan;
  // configure the servo object and assign it to a pin
  _servo.Pin.nbr = pin;
  _servo.Pin.isActive = true;
  _minPulseWidth = minPulseWidth;
  _maxPulseWidth = maxPulseWidth;
  return chan;
}

/**************************************************************************/
/*!
    @brief    Detaches the servo (ledc timer) and de-allocates a servo
              object.
*/
/**************************************************************************/
void ws_ledc_servo::detach() {
  _ledcMgr->detachPin(_servo.Pin.nbr);
  _servo.Pin.isActive = false;
}

/**************************************************************************/
/*!
    @brief    Returns if the servo is attached to a ledc timer
    @returns  True if the servo is attached to a timer, False otherwise.
*/
/**************************************************************************/
bool ws_ledc_servo::attached() { return _servo.Pin.isActive; }

/**************************************************************************/
/*!
    @brief  Writes the pulse width to the connected servo pin.
    @param value  Desired pulse width, in microseconds.
*/
/**************************************************************************/
void ws_ledc_servo::writeMicroseconds(int value) {
  // are we attached to a pin?
  if (!attached()) {
    Serial.println("NOT ATTACHED TO PIN!");
    return;
  }

  // out-of-bounds check, scale value
  if (value < _minPulseWidth)
    value = _minPulseWidth;
  if (value > _maxPulseWidth)
    value = _maxPulseWidth;

  // formula from
  // https://github.com/madhephaestus/ESP32Servo/blob/master/src/ESP32Servo.cpp
  // count = (pulse_high_width / (pulse_period/2**timer_width))
  // 50Hz servo =  20ms pulse_period
  // count = pulse_high_width /((20000/65536))
  uint32_t count = ((double)value / 0.30517578);
  _ledcMgr->setDuty(_servo.Pin.nbr, count);
}
#endif //ARDUINO_FEATHER_ESP32