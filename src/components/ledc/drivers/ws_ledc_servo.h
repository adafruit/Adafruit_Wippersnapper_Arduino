/*!
 * @file ws_ledc_servo.h
 *
 * Driver for ESP32 servo control using the WipperSnapper
 * LEDC peripheral manager API.
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
#ifndef WS_ESP32_SERVO
#define WS_ESP32_SERVO

#include "components/ledc/ws_ledc.h"

// from https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h
#define MIN_PULSE_WIDTH 544  ///< The shortest pulse sent to a servo
#define MAX_PULSE_WIDTH 2400 ///< The longest pulse sent to a servo
#define INVALID_SERVO 255    ///< Flag indicating an invalid servo index

#define DEFAULT_SERVO_FREQ 50 ///< default servo frequency
#define LEDC_TIMER_WIDTH                                                       \
  12 ///< timer width to request from LEDC manager component, in bits (NOTE:
     ///< While ESP32x can go up to 16 bit timer width, ESP32-S2 does not work
     ///< at this resolution. So, for the purposes of keeping this library
     ///< compatible with multiple ESP32x platforms, the timer width has been
     ///< scaled down to 10 bits and the calculation adjusted accordingly)

/** Defines a servo attached to a pin */
typedef struct {
  uint8_t nbr;      ///< Servo's pin number
  uint8_t isActive; ///< True if the servo is enabled
} ServoPin_t;

/** Defines a ws_ledc_servo object */
typedef struct {
  ServoPin_t Pin; ///< Servo properties
} servo_t;

class ws_ledc;

/************************************************************************************************/
/*!
    @brief  High-level driver for servos for ESP32/ESP32-Sx/ESP32-Cx. This
            driver implements a subset of the functions within the Arduino
            servo library,
   (https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h).
*/
/************************************************************************************************/
class ws_ledc_servo {
public:
  ws_ledc_servo();
  ~ws_ledc_servo();
  void setLEDCDriver(ws_ledc *ledcManager);
  // The functions below are compatible with
  // https://github.com/arduino-libraries/Servo/blob/master/src/Servo.h
  uint8_t attach(int pin, int minPulseWidth, int maxPulseWidth, int servoFreq);
  bool attached();
  void detach();
  void writeMicroseconds(int value);

private:
  servo_t _servo;     ///< ws_ledc_servo object
  int _minPulseWidth; ///< Servo's minimum pulse width, in uS.
  int _maxPulseWidth; ///< Servo's maximum pulse width, in uS.
  ws_ledc *_ledcMgr;
};
#endif // WS_ESP32_SERVO