/*!
 * @file src/components/telemetry/hardware.cpp
 *
 * Hardware interface for the telemetry.proto API
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "hardware.h"

// Platform headers used to source the per-target boot/reset reason.
#if defined(ARDUINO_ARCH_ESP32)
// rtc_get_reset_reason() lives in the target-specific ROM header.
#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp32/rom/rtc.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S2)
#include "esp32s2/rom/rtc.h"
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#include "esp32s3/rom/rtc.h"
#elif defined(CONFIG_IDF_TARGET_ESP32C2)
#include "esp32c2/rom/rtc.h"
#elif defined(CONFIG_IDF_TARGET_ESP32C3)
#include "esp32c3/rom/rtc.h"
#elif defined(CONFIG_IDF_TARGET_ESP32C6)
#include "esp32c6/rom/rtc.h"
#elif defined(CONFIG_IDF_TARGET_ESP32H2)
#include "esp32h2/rom/rtc.h"
#else
#include "rom/rtc.h"
#endif

/*!
    @brief  Maps an ESP32 rtc_get_reset_reason() code to a short string.
    @param  reason
            The value returned by rtc_get_reset_reason(coreNum).
    @returns A human-readable reset reason string.
*/
static const char *espCoreResetReasonName(int reason) {
  switch (reason) {
  case 1:
    return "POWERON_RESET"; // Vbat power on reset
  case 3:
    return "SW_RESET"; // Software reset digital core
  case 4:
    return "OWDT_RESET"; // Legacy watch dog reset digital core
  case 5:
    return "DEEPSLEEP_RESET"; // Deep Sleep reset digital core
  case 6:
    return "SDIO_RESET"; // Reset by SLC module, reset digital core
  case 7:
    return "TG0WDT_SYS_RESET"; // Timer Group0 Watch dog reset digital core
  case 8:
    return "TG1WDT_SYS_RESET"; // Timer Group1 Watch dog reset digital core
  case 9:
    return "RTCWDT_SYS_RESET"; // RTC Watch dog Reset digital core
  case 10:
    return "INTRUSION_RESET"; // Intrusion tested to reset CPU
  case 11:
    return "TGWDT_CPU_RESET"; // Time Group reset CPU
  case 12:
    return "SW_CPU_RESET"; // Software reset CPU
  case 13:
    return "RTCWDT_CPU_RESET"; // RTC Watch dog Reset CPU
  case 14:
    return "EXT_CPU_RESET"; // for APP CPU, reset by PRO CPU
  case 15:
    return "RTCWDT_BROWN_OUT_RESET"; // Reset when the vdd voltage is not stable
  case 16:
    return "RTCWDT_RTC_RESET"; // RTC Watch dog reset digital core and rtc
                               // module
  default:
    return "NO_MEAN";
  }
}
#elif defined(ARDUINO_ARCH_SAMD)
/*!
    @brief  Reads the SAMD51 reset cause from the Reset Controller (RSTC).
    @returns A human-readable reset reason string.
*/
static const char *samdResetReasonName() {
#if defined(RSTC)
  uint8_t rcause = RSTC->RCAUSE.reg;
  if (rcause & RSTC_RCAUSE_POR)
    return "PowerOn";
  if (rcause & RSTC_RCAUSE_BODCORE)
    return "BrownOutCore";
  if (rcause & RSTC_RCAUSE_BODVDD)
    return "BrownOutVddcore";
  if (rcause & RSTC_RCAUSE_EXT)
    return "ExtPin";
  if (rcause & RSTC_RCAUSE_WDT)
    return "WDT";
  if (rcause & RSTC_RCAUSE_SYST)
    return "SystemReset";
  if (rcause & RSTC_RCAUSE_BACKUP)
    return "Backup";
  return "Unknown";
#else
  return "Unknown";
#endif
}
#endif

/*!
    @brief  Whether this firmware knows how to source the given telemetry
            metric type (i.e. has a reader for it).
    @param  type
            The telemetry metric type from the broker.
    @returns True if the metric is supported on this device, False otherwise.
*/
bool TelemetryTypeIsSupported(ws_telemetry_Type type) {
  switch (type) {
  case ws_telemetry_Type_TM_RSSI:
  case ws_telemetry_Type_TM_BOOT_REASON:
    return true;
  default:
    // TM_BOOT_COUNT and TM_LATENCY are defined in the API but not yet
    // sourced by this firmware.
    return false;
  }
}

/*!
    @brief  Returns a short human-readable name for a telemetry metric type,
            used for logging and offline SD-card entries.
    @param  type
            The telemetry metric type.
    @returns A static, null-terminated name (e.g. "rssi", "boot_reason").
*/
const char *TelemetryTypeName(ws_telemetry_Type type) {
  switch (type) {
  case ws_telemetry_Type_TM_RSSI:
    return "rssi";
  case ws_telemetry_Type_TM_BOOT_REASON:
    return "boot_reason";
  case ws_telemetry_Type_TM_LATENCY:
    return "latency";
  default:
    return "unknown";
  }
}

/*!
    @brief  TelemetryHardware constructor
    @param  type
            The telemetry metric this instance reports.
    @param  period
            The reporting interval, in seconds. Use 0 to report only once
            at startup.
    @param  report_once
            If true, the device will only report this metric once (via
            Add() or updatePeriod(), i.e. at startup, or when requested).
*/
TelemetryHardware::TelemetryHardware(ws_telemetry_Type type, float period,
                                     bool report_once) {
  did_read_send = false;
  _has_reported_once = false;
  _prv_period = 0;
  _type = type;
  _report_once = report_once;
  SetPeriod(period);
}

/*!
    @brief  TelemetryHardware destructor
*/
TelemetryHardware::~TelemetryHardware() {}

/*!
    @brief  Gets the telemetry metric this instance reports.
    @returns The ws_telemetry_Type for this instance.
*/
ws_telemetry_Type TelemetryHardware::GetType() { return _type; }

/*!
    @brief  Gets a short human-readable name for this instance's metric.
    @returns The metric's name (e.g. "rssi", "boot_reason").
*/
const char *TelemetryHardware::GetName() { return TelemetryTypeName(_type); }

/*!
    @brief  Sets the timer used to report the metric.
    @param  period
            The desired reporting interval, in seconds.
*/
void TelemetryHardware::SetPeriod(float period) {
  _period = period * 1000; // Convert to milliseconds
  _prv_period = 0; // Reset the previous period whenever we set a new one
  if (_report_once)
    _has_reported_once = false; // Reset the reported-once flag if needed
}

/*!
    @brief  Whether the metric reports periodically or only once at startup.
    @returns True if the metric has a non-zero period, False otherwise.
*/
bool TelemetryHardware::IsPeriodic() { return !_report_once; }

/*!
    @brief  Obtains the current time in milliseconds and compares it to the
            last time the metric was reported.
    @returns True if the timer has expired, False otherwise.
*/
bool TelemetryHardware::IsTimerExpired() {
  return millis() - _prv_period > _period;
}

/*!
    @brief  Updates the last time the metric was reported to now.
*/
void TelemetryHardware::MarkPolled() { _prv_period = millis(); }

/*!
    @brief  Whether a report-once (period 0) metric has already been sent.
    @returns True if the metric has already reported once, False otherwise.
*/
bool TelemetryHardware::WasReported() { return _has_reported_once; }

/*!
    @brief  Marks a report-once (period 0) metric as having been sent.
*/
void TelemetryHardware::SetReported() { _has_reported_once = true; }

/*!
    @brief  Reads the current WiFi signal strength (RSSI).
    @returns The RSSI, in dBm.
*/
int32_t TelemetryHardware::ReadRSSI() { return Ws.getRSSI(); }

/*!
    @brief  Reads the reason for the last device reset/boot as a string.
            On ESP32 the reset reason is reported per CPU core (one core
            per line, e.g. "CPU0: ...\nCPU1: ..."), reusing the per-core
            rtc_get_reset_reason() approach. RP2040/RP2350 use the Earle
            Philhower core's chip-level reset reason; SAMD51 reads the RSTC
            reset cause; ESP8266 uses ESP.getResetReason().
    @returns A short human-readable reset reason. May contain multiple
             newline-separated lines on multi-core targets.
*/
const char *TelemetryHardware::ReadBootReason() {
  _boot_reason[0] = '\0';
#if defined(ARDUINO_ARCH_ESP32)
  // ESP32 exposes a reset reason per CPU core - report one CPU per line.
  // ESP.getChipCores() returns a uint8_t.
  uint8_t num_cores = ESP.getChipCores();
  if (num_cores < 1)
    num_cores = 1;
  for (uint8_t core = 0; core < num_cores; core++) {
    // Separate cores with a newline (no leading newline on the first core).
    const char *separator = "\n";
    if (core == 0)
      separator = "";
    // A single-CPU part gets the full SINGLE_BOOT_REASON budget. When
    // multi-core, cap each line so every CPU's reason fits within
    // _boot_reason (i.e. 48 chars per line when dual-core).
    char line[SINGLE_BOOT_REASON];
    size_t max_line = sizeof(line);
    if (num_cores > 1)
      max_line = BOOT_REASONS_LEN / num_cores;
    snprintf(line, max_line, "%sCPU%u: %s", separator, core,
             espCoreResetReasonName((int)rtc_get_reset_reason(core)));
    strncat(_boot_reason, line,
            sizeof(_boot_reason) - strlen(_boot_reason) - 1);
  }
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RP2350)
  const char *reason;
  switch (rp2040.getResetReason()) {
  case RP2040::resetReason_t::PWRON_RESET:
    reason = "PowerOn";
    break;
  case RP2040::resetReason_t::RUN_PIN_RESET:
    reason = "RunPin";
    break;
  case RP2040::resetReason_t::SOFT_RESET:
    reason = "Reboot";
    break;
  case RP2040::resetReason_t::WDT_RESET:
    reason = "WDT";
    break;
  case RP2040::resetReason_t::DEBUG_RESET:
    reason = "Debug";
    break;
  case RP2040::resetReason_t::GLITCH_RESET:
    reason = "Glitch";
    break;
  case RP2040::resetReason_t::BROWNOUT_RESET:
    reason = "BrownOut";
    break;
  case RP2040::resetReason_t::UNKNOWN_RESET:
  default:
    reason = "Unknown";
    break;
  }
  strncpy(_boot_reason, reason, sizeof(_boot_reason) - 1);
  _boot_reason[sizeof(_boot_reason) - 1] = '\0';
#elif defined(ARDUINO_ARCH_ESP8266)
  strncpy(_boot_reason, ESP.getResetReason().c_str(), sizeof(_boot_reason) - 1);
  _boot_reason[sizeof(_boot_reason) - 1] = '\0';
#elif defined(ARDUINO_ARCH_SAMD)
  strncpy(_boot_reason, samdResetReasonName(), sizeof(_boot_reason) - 1);
  _boot_reason[sizeof(_boot_reason) - 1] = '\0';
#else
  strncpy(_boot_reason, "Unknown", sizeof(_boot_reason) - 1);
  _boot_reason[sizeof(_boot_reason) - 1] = '\0';
#endif
  return _boot_reason;
}
