/*!
 * @file src/components/pwm/controller.h
 *
 * Controller for the pwm API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2025 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_PWM_CONTROLLER_H
#define WS_PWM_CONTROLLER_H
#include "hardware.h"
#include "model.h"
#include "wippersnapper.h"
#define MAX_PWM_PINS 25 ///< Maximum number of PWM pins supported

class wippersnapper; // Forward declaration
class PWMModel;      // Forward declaration
class PWMHardware;   // Forward declaration

/*!
    @brief  Routes messages using the pwm.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's PWM pins.
*/
class PWMController {
public:
  PWMController();
  ~PWMController();
  bool Router(pb_istream_t *stream);
  bool Handle_PWM_Add(ws_pwm_Add *msg);
  bool Handle_PWM_Write(ws_pwm_Write *msg);
  bool Handle_PWM_Remove(ws_pwm_Remove *msg);
  int GetPWMHardwareIdx(uint8_t pin);

private:
  PWMModel *_pwm_model;
  PWMHardware *_pwm_hardware[MAX_PWM_PINS] = {nullptr};
  int _active_pwm_pins;
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance
#endif                   // WS_PWM_CONTROLLER_H