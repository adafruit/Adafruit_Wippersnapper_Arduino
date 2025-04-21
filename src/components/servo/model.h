/*!
 * @file src/components/servo/model.h
 *
 * Model for the servo.proto message.
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
#ifndef WS_SERVO_MODEL_H
#define WS_SERVO_MODEL_H
#include "Wippersnapper_V2.h"

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from servo.proto.
*/
/**************************************************************************/
class ServoModel {
public:
  ServoModel();
  ~ServoModel();
  bool DecodeServoAdd(pb_istream_t *stream);
  wippersnapper_servo_ServoAdd *GetServoAddMsg();
  bool EncodeServoAdded(char *pin_name, bool did_attach);
  wippersnapper_servo_ServoAdded *GetServoAddedMsg();
  bool DecodeServoRemove(pb_istream_t *stream);
  wippersnapper_servo_ServoRemove *GetServoRemoveMsg();
  bool DecodeServoWrite(pb_istream_t *stream);
  wippersnapper_servo_ServoWrite *GetServoWriteMsg();
private:
  wippersnapper_servo_ServoAdd _msg_servo_add; ///< ServoAdd message object
  wippersnapper_servo_ServoAdded _msg_servo_added; ///< ServoAdded message object
  wippersnapper_servo_ServoRemove _msg_servo_remove; ///< ServoRemove message object
  wippersnapper_servo_ServoWrite _msg_servo_write; ///< ServoWrite message object
};
#endif // WS_SERVO_MODEL_H