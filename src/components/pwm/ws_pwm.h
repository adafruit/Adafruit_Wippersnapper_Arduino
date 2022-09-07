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
#ifndef WS_PWM
#define WS_PWM

#include "components/ledc/ws_ledc.h"

class Wippersnapper;

class ws_ledc;

/**************************************************************************/
/*!
    @brief  Interface for WipperSnapper PWM
*/
/**************************************************************************/
class ws_pwm {
public:
  ws_pwm(ws_ledc *ledcManager);
  ~ws_pwm();

  bool attach(uint8_t pin, double freq, uint8_t resolution);
  void detach(uint8_t pin);

  void writeDutyCycle(uint8_t pin, int dutyCycle);
  void writeTone(uint8_t pin, uint32_t freq);
  void noTone(uint8_t pin);

private:
  ws_ledc *_ledcMgr; ///< pointer to ws_ledc
};
extern Wippersnapper WS;

#endif // WS_PWM