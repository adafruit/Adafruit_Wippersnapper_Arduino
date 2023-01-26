/*!
 * @file ws_servo.h
 *
 * High-level interface for wippersnapper to manage servo objects
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 *
 * Brent Rubell for Adafruit Industries 2022
 *
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SERVO
#define WS_SERVO

#include "Wippersnapper.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "components/ledc/drivers/servo/ws_ledc_servo.h"
#else
#include <Servo.h>
#endif

#ifdef ARDUINO_ARCH_RP2040
#define MAX_SERVO_NUM                                                          \
  8 ///< Maximum number of servo objects for Pico,
    ///< https://arduino-pico.readthedocs.io/en/latest/servo.html
#else
#define MAX_SERVO_NUM 16 ///< Maximum number of servo objects
#endif

#define MIN_SERVO_PULSE_WIDTH 500 ///< Default min. servo pulse width of 500uS
#define ERR_SERVO_ATTACH 255      ///< Error when attempting to attach servo

#if defined(ARDUINO_ARCH_ESP32)
class ws_ledc_servo;
/** Servo object for ESP32-servo implementation */
struct servoComponent {
  ws_ledc_servo *servoObj = nullptr; ///< Servo object
  uint8_t pin = 0;                   ///< Servo's pin number
};
#else
/** Servo object for Generic servo implementation */
struct servoComponent {
  Servo *servoObj = nullptr; ///< Servo object
  uint8_t pin = 0;           ///< Servo's pin number
};
#endif

class Wippersnapper;

/**************************************************************************/
/*!
    @brief  Interface for WipperSnapper servo control
*/
/**************************************************************************/
class ws_servo {
public:
  ws_servo(){};
  ~ws_servo();
  bool servo_attach(int pin, int minPulseWidth, int maxPulseWidth, int freq);
  void servo_detach(int pin);
  void servo_write(int pin, int value);
  servoComponent *getServoComponent(uint8_t pin);

private:
  servoComponent _servos[MAX_SERVO_NUM]; ///< Container of servo objects and
                                         ///< their associated pin #s
};
extern Wippersnapper WS;

#endif // WS_SERVO