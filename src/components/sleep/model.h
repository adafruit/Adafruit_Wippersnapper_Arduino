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
  ws_sleep_SleepConfig *GetSleepConfig();
  bool HasExt0();
  const ws_sleep_Ext0Config *GetExt0Config();
  ws_sleep_Goodnight *GetSleepGoodnight();
  bool DecodeSleepConfig(pb_istream_t *stream);
  bool EncodeSleepGoodnight(const char *msg);
  void SetSleepEnterTimer(bool lock, const char *mode, uint32_t run_duration,
                          uint32_t timer_duration);
  void SetSleepEnterExt0(bool lock, const char *mode, uint32_t run_duration,
                         const char *pin_name, bool pin_level);
  uint32_t getRunDurationMs();

private:
  void ConvertSleepMode(const char *mode_str, ws_sleep_SleepMode &mode);
  ws_sleep_Goodnight _msg_sleep_goodnight; ///< Goodnight message object
  ws_sleep_SleepConfig _msg_sleep_enter;   ///< Enter message object
  ws_sleep_Ext0Config _ext0_config; ///< Ext0 wakeup config (SleepConfig.ext0s
                                    ///< is an undecoded callback field)
  bool _has_ext0 = false;           ///< True if _ext0_config holds a config
  uint32_t _run_duration = 60; ///< Duration to run before sleeping, in seconds
};

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_MODEL_H