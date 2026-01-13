/*!
 * @file src/components/sleep/hardware.h
 *
 * Hardware implementation for the sleep.proto message.
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
#ifndef WS_SLEEP_HARDWARE_H
#define WS_SLEEP_HARDWARE_H

#ifdef ARDUINO_ARCH_ESP32
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wippersnapper.h"
#include <sys/time.h>
#include <time.h>

/*!
    @brief  Interface for interacting with hardware's sleep functionality.
*/
class SleepHardware {
public:
  SleepHardware();
  ~SleepHardware();
  bool EnableDeepSleep(int duration);
  bool EnableLightSleep(int duration);
  bool CheckBootButton();
  bool RegisterRTCTimerWakeup(uint64_t duration);
  bool RegisterExt0Wakeup(const char *pin_name, bool pin_level, bool pin_pull);
  void DisableExternalComponents();
  ws_sleep_EspWakeCause GetEspWakeCauseEnum();
  esp_sleep_source_t GetEspSleepSource();
  ws_sleep_SleepMode GetSleepMode();
  int GetSleepDuration();
  void SetSleepEnterTime();
private:
  void GetSleepWakeupCause();
  void CalculateSleepDuration();
  esp_sleep_source_t
      _wakeup_cause; ///< Sleep wakeup cause, obtained during class construction
  int _sleep_time;   ///< Time spent sleeping, in seconds
};

#endif // ARDUINO_ARCH_ESP32
#endif // WS_SLEEP_HARDWARE_H