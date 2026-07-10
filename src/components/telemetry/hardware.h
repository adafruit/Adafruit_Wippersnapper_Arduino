/*!
 * @file src/components/telemetry/hardware.h
 *
 * Hardware implementation for the telemetry.proto API
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
#ifndef WS_TELEMETRY_HARDWARE_H
#define WS_TELEMETRY_HARDWARE_H
#include "wippersnapper.h"

/*!
    @brief  The metric a telemetry component instance reports. Resolved from
            the instance's name (e.g. "rssi", "boot_reason").
*/
typedef enum {
  TELEMETRY_KIND_UNKNOWN = 0, ///< Unrecognized telemetry metric name
  TELEMETRY_KIND_RSSI,        ///< WiFi signal strength (RSSI), in dBm
  TELEMETRY_KIND_BOOT_REASON, ///< Reason for the last device reset/boot
} TelemetryKind;

/*!
    @brief  Represents a single telemetry metric instance and provides the
            platform-specific reads used to source its value.
*/
class TelemetryHardware {
public:
  TelemetryHardware(const char *name, float period);
  ~TelemetryHardware();
  const char *GetName();
  TelemetryKind GetKind();
  void SetPeriod(float period);
  bool IsPeriodic();
  bool IsTimerExpired();
  void MarkPolled();
  bool WasReported();
  void SetReported();
  // Metric reads
  int32_t ReadRSSI();
  const char *ReadBootReason();
  bool did_read_send; ///< True if the last read was sent to IO, False otherwise
private:
  char _name[24];        ///< Telemetry component name (e.g. "rssi")
  TelemetryKind _kind;   ///< Metric kind resolved from _name
  float _period;         ///< Desired reporting interval, in milliseconds
  float _prv_period;     ///< Last time the metric was reported, in milliseconds
  bool _reported_once;   ///< True once a report-once (period 0) metric has sent
  char _boot_reason[96]; ///< Backing store for the assembled boot reason string
};
#endif // WS_TELEMETRY_HARDWARE_H
