/*!
 * @file src/components/telemetry/controller.h
 *
 * Controller for the telemetry.proto API
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
#ifndef WS_TELEMETRY_CONTROLLER_H
#define WS_TELEMETRY_CONTROLLER_H
#include "hardware.h"
#include "model.h"
#include "wippersnapper.h"

class wippersnapper;     ///< Forward declaration
class TelemetryModel;    ///< Forward declaration
class TelemetryHardware; ///< Forward declaration

/*!
    @brief  Routes messages between the telemetry.proto API and the device's
            metric sources (e.g. RSSI, boot reason).
*/
class TelemetryController {
public:
  TelemetryController();
  ~TelemetryController();
  // Routing
  bool Router(pb_istream_t *stream);
  bool Handle_TelemetryAdd(ws_telemetry_Add *msg);
  bool Handle_TelemetryRemove(ws_telemetry_Remove *msg);
  // Polling
  void update(bool force = false);
  bool UpdateComplete();
  void ResetFlags();

private:
  bool updatePeriod(ws_telemetry_Type type, float period);
  bool encodeAndPublish();
  bool Publish(ws_telemetry_Type type, ws_sensor_Type value_type, float value);
  bool Publish(ws_telemetry_Type type, ws_sensor_Type value_type,
               const char *value);
  TelemetryModel *_telemetry_model; ///< telemetry model
  std::vector<TelemetryHardware *> _telemetry_metrics;
};
extern wippersnapper *Ws; ///< Wippersnapper V2 instance
#endif                    // WS_TELEMETRY_CONTROLLER_H
