/*!
 * @file ws_servo.cpp
 *
 * High-level servo manager interface for WipperSnapper.
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
#include "ws_servo.h"

/**************************************************************************/
/*!
    @brief  Constructor
*/
/**************************************************************************/
ws_servo::ws_servo() {}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_servo::~ws_servo() {
  for (int i = 0; i < MAX_SERVO_NUM; i++) {
    // de-allocate servo pins, if attached
    if (_servos[i].servoObj->attached())
      _servos[i].servoObj->detach();
  }
}

/**************************************************************************/
/*!
    @brief   Obtains the index of the requested servo object within the
             array of servos.
    @param   servoPin
             The desired servo pin.
    @returns The index of the servo object, -1 otherwise.
*/
/**************************************************************************/
int ws_servo::getServoIdx(uint8_t servoPin) {
  int servoIdx = -1;
  for (int i = 0; i < MAX_SERVO_NUM; i++) {
    if (_servos[i].pin == servoPin)
      return i;
  }
  return servoIdx;
}

/**************************************************************************/
/*!
    @brief    Attaches a servo object to a pin.
    @param    pin            Desired GPIO pin.
    @param    minPulseWidth  Minimum pulsewidth, in uS.
    @param    maxPulseWidth  Maximum pulsewidth, in uS.
    @param    freq           Servo Frequency, default is 50Hz
    @returns  True if a servo is successfully attached to a pin,
              False otherwise
*/
/**************************************************************************/
bool ws_servo::servo_attach(int pin, int minPulseWidth, int maxPulseWidth,
                            int freq) {
#ifdef ARDUINO_ARCH_ESP32
  // ESP32/x specific implementation
  ws_ledc_servo *servo = new ws_ledc_servo();
  servo->setLEDCDriver(WS._ledc);
#else
  // generic Servo.h
  Servo *servo = new Servo();
#endif

  uint16_t rc = 255;
#ifdef ARDUINO_ARCH_ESP32
  rc = servo->attach(pin, minPulseWidth, maxPulseWidth, freq);
#else
  rc = servo->attach(pin, minPulseWidth, maxPulseWidth);
#endif
  if (rc == 255)
    return false; // allocation or pin error

  // Attempt to allocate an unused servo
  int servoIdx = -1;
  for (int i = 0; i < MAX_SERVO_NUM; i++) {
    Serial.println(_servos[i].pin);
    if (_servos[i].pin == 255) {
      servoIdx = i;
      Serial.print("Servos IDX:");
      Serial.println(servoIdx);
      break;
    }
  }
  // check if servo was allocated
  if (servoIdx == 255) {
    Serial.println("ERROR: Maximum servos reached!");
    return false;
  }

  // create a new servo component storage struct
  _servos[servoIdx].servoObj = servo;
  _servos[servoIdx].pin = pin;

  // Write the default minimum to a servo
  servo_write(_servos[servoIdx].pin, MIN_SERVO_PULSE_WIDTH);
  return true;
}

/**************************************************************************/
/*!
    @brief    Detaches a servo from a pin and re-allocates the GPIO pin.
    @param    pin  Desired GPIO pin.
*/
/**************************************************************************/
void ws_servo::servo_detach(int pin) {
  int servoIdx = getServoIdx(pin);
  if (servoIdx == -1) {
    WS_DEBUG_PRINTLN("ERROR: Can not detach servo, not found!");
    return;
  }
  // default value for unallocated servo pin
  _servos[servoIdx].pin = 255;
  // detach the servo from the pin
  _servos[servoIdx].servoObj->detach();
  // delete the object
  delete _servos[servoIdx].servoObj;
}

/**************************************************************************/
/*!
    @brief    Writes a pulse width to a servo pin.
    @param    pin    Desired GPIO pin.
    @param    value  Desired pulse width, in uS.
*/
/**************************************************************************/
void ws_servo::servo_write(int pin, int value) {
  int servoIdx = getServoIdx(pin);
  if (servoIdx == -1) {
    WS_DEBUG_PRINTLN("ERROR: Can not detach servo, not found!");
    return;
  }
  // write to servo
  _servos[servoIdx].servoObj->writeMicroseconds(value);
}