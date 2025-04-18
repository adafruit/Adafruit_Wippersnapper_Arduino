/*!
 * @file src/components/pwm/model.h
 *
 * Model for the pwm.proto message.
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
#ifndef WS_PWM_MODEL_H
#define WS_PWM_MODEL_H
#include "Wippersnapper_V2.h"

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from pwm.proto.
*/
/**************************************************************************/
class PWMModel {
public:
  PWMModel();
  ~PWMModel();
  bool DecodePWMAdd(pb_istream_t *stream);
  wippersnapper_pwm_PWMAdd *GetPWMAddMsg();
  bool EncodePWMAdded(char *pin_name, bool did_attach);
  wippersnapper_pwm_PWMAdded *GetPWMAddedMsg();
  bool DecodePWMRemove(pb_istream_t *stream);
  wippersnapper_pwm_PWMRemove *GetPWMRemoveMsg();
  bool DecodePWMWriteDutyCycle(pb_istream_t *stream);
  wippersnapper_pwm_PWMWriteDutyCycle *GetPWMWriteDutyCycleMsg();
  bool DecodePWMWriteDutyCycleMulti(pb_istream_t *stream);
  wippersnapper_pwm_PWMWriteDutyCycleMulti *GetPWMWriteDutyCycleMultiMsg();
  bool DecodePWMWriteFrequency(pb_istream_t *stream);
  wippersnapper_pwm_PWMWriteFrequency *GetPWMWriteFrequencyMsg();
private:
  wippersnapper_pwm_PWMAdd _msg_pwm_add; ///< PWMAdd message object
  wippersnapper_pwm_PWMAdded _msg_pwm_added; ///< PWMAdded message object
  wippersnapper_pwm_PWMRemove _msg_pwm_remove; ///< PWMRemove message object
  wippersnapper_pwm_PWMWriteDutyCycle _msg_pwm_write_duty_cycle; ///< PWMWriteDutyCycle message object
  wippersnapper_pwm_PWMWriteDutyCycleMulti _msg_pwm_write_duty_cycle_multi; ///< PWMWriteDutyCycleMulti message object
  wippersnapper_pwm_PWMWriteFrequency _msg_pwm_write_frequency; ///< PWMWriteFrequency message object
};
#endif // WS_PWM_MODEL_H