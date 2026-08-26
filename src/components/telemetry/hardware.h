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

/// Max length of one CPU's boot reason (the entire string on a
/// single-CPU part). Longest line is "\nCPUn: RTCWDT_BROWN_OUT_RESET"
#define SINGLE_BOOT_REASON 64
/// Max total length of all CPUs' boot reasons, split evenly per CPU when
/// multi-core. Keep in sync with the identical define in
/// telemetry/model.h.
#define BOOT_REASONS_LEN 96

/*!
    @brief  Whether this firmware knows how to source the given telemetry
            metric type (i.e. has a reader for it).
    @param  type
            The telemetry metric type from the broker.
    @returns True if the metric is supported on this device, False otherwise.
*/
bool TelemetryTypeIsSupported(ws_telemetry_Type type);

/*!
    @brief  Returns a short human-readable name for a telemetry metric type,
            used for logging and offline SD-card entries.
    @param  type
            The telemetry metric type.
    @returns A static, null-terminated name (e.g. "rssi", "boot_reason").
*/
const char *TelemetryTypeName(ws_telemetry_Type type);

/*!
    @brief  Represents a single telemetry metric instance and provides the
            platform-specific reads used to source its value.
*/
class TelemetryHardware {
public:
  TelemetryHardware(ws_telemetry_Type type, float period, bool report_once);
  ~TelemetryHardware();
  ws_telemetry_Type GetType();
  const char *GetName();
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
  ws_telemetry_Type _type; ///< The metric this instance reports
  float _period;           ///< Desired reporting interval, in milliseconds
  float _prv_period;       ///< Last time the metric was reported, in ms
  bool _report_once;       ///< True if the metric should be reported only once
  bool _has_reported_once; ///< True once a report-once (period 0) metric has
                           ///< sent
  char _boot_reason[BOOT_REASONS_LEN]; ///< Backing store for the assembled boot
                                       ///< reason string
};
#endif // WS_TELEMETRY_HARDWARE_H
