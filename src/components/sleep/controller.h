/*!
 * @file src/components/sleep/controller.h
 *
 * Controller for the Sleep API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_SLEEP_CONTROLLER_H
#define WS_SLEEP_CONTROLLER_H

#ifdef ARDUINO_ARCH_ESP32
#include "hardware.h"
#include "model.h"
#include "wippersnapper.h"

class wippersnapper; ///< Forward declaration
class SleepModel;    ///< Forward declaration
class SleepHardware; ///< Forward declaration

/*!
    @brief  Routes messages using the sleep.proto API to the
            appropriate hardware and model classes, controls and tracks
            the state of the hardware's sleep functionality.
*/
class SleepController {
public:
  SleepController();
  ~SleepController();
  SleepModel *GetModel();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_Sleep_Enter(ws_sleep_Enter *msg);
  // Hardware-related getters
  ws_sleep_EspWakeCause GetEspWakeCause();
  int GetSleepDuration();
  void CheckBootButton();
  bool DidWakeFromSleep();
  bool IsSleepMode();
  void HandleNetFSMFailure();

private:
  bool ConfigureDeepSleep(const ws_sleep_Enter *msg);
  // TODO: Add ConfigureLightSleep() funcs here
  SleepModel *_sleep_model;       ///< Sleep model
  SleepHardware *_sleep_hardware; ///< Sleep hardware
  ws_sleep_SleepMode _sleep_mode; ///< Current sleep mode
  bool _btn_cfg_mode; ///< Value of BOOT button during class construction
  bool _lock;         ///< Whether the sleep configuration is locked
  bool _has_ext_pwr_components; ///< Whether externally powered components are
                                ///< present (i.e: tft, i2c, neopixel, etc)
};
extern wippersnapper Ws; ///< Wippersnapper V2 instance

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_CONTROLLER_H