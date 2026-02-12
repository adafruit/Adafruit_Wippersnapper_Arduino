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

#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_RP2350)

#ifdef ARDUINO_ARCH_ESP32
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <sys/time.h>
#include <time.h>
#endif

#include "wippersnapper.h"

/*!
    @brief  Interface for interacting with hardware's sleep functionality.
*/
class SleepHardware {
public:
  SleepHardware();
  ~SleepHardware();
  void DisableExternalComponents();
  void ReenableExternalComponents();
  const char *GetWakeupReasonName();
#ifdef ARDUINO_ARCH_ESP32
  bool RegisterRTCTimerWakeup(uint64_t duration);
  bool RegisterExt0Wakeup(const char *pin_name, bool pin_level, bool pin_pull);
  ws_sleep_EspWakeCause GetEspWakeCauseEnum();
  esp_sleep_source_t GetEspSleepSource();
  bool StopWiFi();
  bool RestoreWiFi();
#endif
  ws_sleep_SleepMode GetSleepMode();
  void SetSleepMode(ws_sleep_SleepMode mode);
  void CalculateSleepDuration();
  int GetSleepDuration();
  void SetSleepEnterTime();
  void GetSleepWakeupCause();
  // Storage API for RTC timestamps
  uint32_t GetPrvSoftRtcCounter();
  void StoreSoftRtcCounter(uint32_t counter);
  uint32_t GetSleepCycleCount();
  // Storage API for log filename persistence
  void StoreLogFilename(const char *filename);
  const char *GetLogFilename();

private:
  int _sleep_time; ///< Time spent sleeping, in seconds
#ifdef ARDUINO_ARCH_ESP32
  esp_sleep_source_t
      _wakeup_cause; ///< Sleep wakeup cause, obtained during class construction
  // NVS helpers (for chips without RTC memory)
  bool NvsReadU32(const char *key, uint32_t *value);
  bool NvsWriteU32(const char *key, uint32_t value);
  bool NvsReadI32(const char *key, int32_t *value);
  bool NvsWriteI32(const char *key, int32_t value);
  bool NvsReadStr(const char *key, char *value, size_t max_len);
  bool NvsWriteStr(const char *key, const char *value);
#endif
#ifdef ARDUINO_ARCH_RP2350
  bool _did_wake_from_sleep; ///< Whether device woke from sleep (tracked via
                             ///< ws_wdt)
#endif
};

#endif // ARDUINO_ARCH_ESP32 || ARDUINO_ARCH_RP2350
#endif // WS_SLEEP_HARDWARE_H