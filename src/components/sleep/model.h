/*!
 * @file src/components/sleep/model.h
 *
 * Model interface for the sleep.proto message.
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
#ifndef WS_SLEEP_MODEL_H
#define WS_SLEEP_MODEL_H

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2350)
#include "wippersnapper.h"

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from sleep.proto.
*/
class SleepModel {
public:
  SleepModel();
  ~SleepModel();
  ws_sleep_Enter *GetSleepEnterMsg();
  ws_sleep_Goodnight *GetSleepGoodnightMsg();
  ws_sleep_Wake *GetSleepWakeMsg();
  bool DecodeSleepEnter(pb_istream_t *stream);
  bool EncodeSleepGoodnight(const char *msg);
  bool EncodeSleepWake(ws_sleep_EspWakeCause cause, uint32_t sleep_duration);
  void SetSleepEnterTimer(bool lock, const char *mode, uint32_t run_duration,
                          uint32_t timer_duration);
  void SetSleepEnterExt0(bool lock, const char *mode, uint32_t run_duration,
                         const char *pin_name, bool pin_level, bool pin_pull);
  uint32_t GetRunDuration();

private:
  void ConvertSleepMode(const char *mode_str, ws_sleep_SleepMode &mode);
  ws_sleep_Goodnight _msg_sleep_goodnight; ///< Goodnight message object
  ws_sleep_Wake _msg_sleep_wake;           ///< Wake message object
  ws_sleep_Enter _msg_sleep_enter;         ///< Enter message object
  uint32_t _run_duration = 60; ///< Duration to run before sleeping, in seconds
};

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_MODEL_H