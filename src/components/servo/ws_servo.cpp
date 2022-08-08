/*!
 * @file ws_component_servo.cpp
 *
 * High-level servo manager interface for WipperSnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section author Author
 *
 * Written by Brent Rubell for Adafruit Industries, 2022.
 *
 *
 * @section license License
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
ws_servo::ws_servo() {
/*   for (int i = 0; i < MAX_SERVO_NUM; i++) {
    _servos[i].pin = 255;
    _servos[i].servoObj = nullptr;
  } */
}

/**************************************************************************/
/*!
    @brief  Destructor
*/
/**************************************************************************/
ws_servo::~ws_servo() {
/*   for (int i = 0; i < MAX_SERVO_NUM; i++) {
    // de-allocate servo pins, if attached
    if (_servos[i].servoObj->attached())
      _servos[i].servoObj->detach();
  } */
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
bool ws_servo::servo_attach(int pin, int minPulseWidth,
                                      int maxPulseWidth, int freq) {
  // ESP32-specific
  ws_ledc_servo *servo = new ws_ledc_servo();
  servo->setLEDCDriver(WS._ledc);
   

  // generalized
  uint16_t rc = servo->attach(pin, minPulseWidth, maxPulseWidth, freq);
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
  // check if allocated
  if (servoIdx == 255) {
    Serial.println("ERROR: Maximum servos reached!");
    return false;
  }

  // create a new servo component storage struct
  _servos[servoIdx].servoObj = servo;
  _servos[servoIdx].pin = pin;

  // call attach pin
  _servos[servoIdx].servoObj->attach(pin, minPulseWidth, maxPulseWidth, freq);
  return true;
}
