/*!
 * @file src/components/servo/hardware.h
 *
 * Hardware for the servo.proto message.
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
#ifndef WS_SERVO_HARDWARE_H
#define WS_SERVO_HARDWARE_H
#include "Wippersnapper_V2.h"


/**************************************************************************/
/*!
    @brief  Interface for interacting with hardware's Servo API.
*/
/**************************************************************************/
class ServoHardware {
public:
  ServoHardware();
  ~ServoHardware();
private:
};
#endif // WS_SERVO_HARDWARE_H