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
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"

class Wippersnapper_V2; // Forward declaration
class PWMModel;         // Forward declaration
class PWMHardware;      // Forward declaration

/**************************************************************************/
/*!
    @brief  Routes messages using the pwm.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's PWM pins.
*/
/**************************************************************************/
class PWMController {
public:
  PWMController();
  ~PWMController();
  // Called by the cbDecodeBrokerToDevice router function
  bool Handle_PWM_Add(pb_istream_t *stream);
  bool Handle_PWM_Write_DutyCycle(pb_istream_t *stream);
  bool Handle_PWM_Write_DutyCycle_Multi(pb_istream_t *stream);
  bool Handle_PWM_Write_Frequency(pb_istream_t *stream);
  bool Handle_PWM_Remove(pb_istream_t *stream);

private:
  PWMModel *_pwm_model;
  PWMHardware *_pwm_hardware;
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_PWM_CONTROLLER_H