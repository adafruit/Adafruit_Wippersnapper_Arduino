/*!
 * @file src/components/servo/controller.h
 *
 * Controller for the servo API
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
#ifndef WS_SERVO_CONTROLLER_H
#define WS_SERVO_CONTROLLER_H
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"

#ifdef ARDUINO_ARCH_RP2040
#define MAX_SERVOS                                                             \
  8 ///< Maximum number of servo objects for RP2040,
    ///< https://arduino-pico.readthedocs.io/en/latest/servo.html
#else
#define MAX_SERVOS 16 ///< Maximum number of servo objects
#endif
#define MIN_SERVO_PULSE_WIDTH 500 ///< Default min. servo pulse width of 500uS

class Wippersnapper_V2; // Forward declaration
class ServoModel;       // Forward declaration
class ServoHardware;    // Forward declaration

/**************************************************************************/
/*!
    @brief  Routes messages using the servo.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's Servo pins.
*/
/**************************************************************************/
class ServoController {
public:
  ServoController();
  ~ServoController();
  bool Handle_Servo_Add(pb_istream_t *stream);
  bool Handle_Servo_Write(pb_istream_t *stream);
  bool Handle_Servo_Remove(pb_istream_t *stream);

private:
  ServoModel *_servo_model;
  ServoHardware *_servo_hardware[MAX_SERVOS] = {nullptr};
  int _active_servo_pins; ///< Number of active servo pins
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_SERVO_CONTROLLER_H