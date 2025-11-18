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

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from pwm.proto.
*/
class PWMModel {
public:
  PWMModel();
  ~PWMModel();
  bool DecodePWMAdd(pb_istream_t *stream);
  ws_pwm_Add *GetPWMAddMsg();
  bool EncodePWMAdded(char *pin_name, bool did_attach);
  ws_pwm_Added *GetPWMAddedMsg();
  bool DecodePWMRemove(pb_istream_t *stream);
  ws_pwm_Remove *GetPWMRemoveMsg();
  bool DecodePWMWrite(pb_istream_t *stream);
  ws_pwm_Write *GetPWMWriteMsg();

private:
  ws_pwm_Add _msg_pwm_add;       ///< PWMAdd message object
  ws_pwm_Added _msg_pwm_added;   ///< PWMAdded message object
  ws_pwm_Remove _msg_pwm_remove; ///< PWMRemove message object
  ws_pwm_Write _msg_pwm_write;   ///< PWMWrite message object
};
#endif // WS_PWM_MODEL_H