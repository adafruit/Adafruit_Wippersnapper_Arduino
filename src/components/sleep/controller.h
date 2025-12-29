/*!
 * @file src/components/sleep/controller.h
 *
 * Controller for the Sleep API
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
#ifndef WS_SLEEP_CONTROLLER_H
#define WS_SLEEP_CONTROLLER_H

#ifdef ARDUINO_ARCH_ESP32
#include "Wippersnapper_V2.h"
#include "hardware.h"
#include "model.h"

class Wippersnapper_V2; ///< Forward declaration
class SleepModel;       ///< Forward declaration
class SleepHardware;    ///< Forward declaration

/*!
    @brief  Routes messages using the sleep.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's sleep functionality.
*/
class SleepController {
public:
  SleepController();
  ~SleepController();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_Sleep_Enter(ws_sleep_Enter *msg);
  // Helper functions
  void CheckBootButton();
  SleepModel *GetModel();
private:
  bool ConfigureDeepSleep(ws_sleep_TimerConfig cfg_timer);
  bool ConfigureDeepSleep(ws_sleep_PinConfig cfg_pin);
  // TODO: Add ConfigureLightSleep() funcs here
  SleepModel *_sleep_model;       ///< Sleep model
  SleepHardware *_sleep_hardware; ///< Sleep hardware
  bool _btn_cfg_mode; ///< Value of BOOT button during class construction
  bool _lock;      ///< Whether the sleep configuration is locked
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_CONTROLLER_H