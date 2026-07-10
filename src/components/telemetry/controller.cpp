/*!
 * @file src/components/telemetry/controller.cpp
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
#include "controller.h"

/*!
    @brief  TelemetryController constructor
*/
TelemetryController::TelemetryController() {
  _telemetry_model = new TelemetryModel();
}

/*!
    @brief  TelemetryController destructor
*/
TelemetryController::~TelemetryController() {
  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    delete _telemetry_metrics[i];
  }
  _telemetry_metrics.clear();
  delete _telemetry_model;
}

/*!
    @brief  Routes messages using the telemetry.proto API to the
            appropriate controller functions.
    @param  stream
            The nanopb input stream.
    @return True if the message was successfully routed, False otherwise.
*/
bool TelemetryController::Router(pb_istream_t *stream) {
  // Attempt to decode the telemetry B2D envelope
  ws_telemetry_B2D b2d = ws_telemetry_B2D_init_zero;
  if (!ws_pb_decode(stream, ws_telemetry_B2D_fields, &b2d)) {
    WS_DEBUG_PRINTLN(
        "[telemetry] ERROR: Unable to decode telemetry B2D envelope");
    return false;
  }

  // Route based on payload type
  bool res = false;
  switch (b2d.which_payload) {
  case ws_telemetry_B2D_add_tag:
    res = Handle_TelemetryAdd(&b2d.payload.add);
    break;
  case ws_telemetry_B2D_remove_tag:
    res = Handle_TelemetryRemove(&b2d.payload.remove);
    break;
  default:
    WS_DEBUG_PRINTLN("[telemetry] WARNING: Unsupported telemetry payload");
    res = false;
    break;
  }

  return res;
}

/*!
    @brief  Handles a TelemetryAdd message from the broker. Creates (or
            updates) a telemetry metric instance and starts reporting it.
    @param  msg
            The TelemetryAdd message.
    @return True if the metric was successfully registered (or an unknown
            metric name was gracefully ignored), False otherwise.
*/
bool TelemetryController::Handle_TelemetryAdd(ws_telemetry_Add *msg) {
  WS_DEBUG_PRINT("[telemetry] Handle_TelemetryAdd: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  // Validate the metric name
  if (strlen(msg->name) == 0) {
    Ws.error_handler->publishComponentError(msg->name,
                                            "Empty telemetry metric name");
    return false;
  }

  // If a metric with this name already exists, update its period instead of
  // registering a duplicate.
  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    if (strcmp(_telemetry_metrics[i]->GetName(), msg->name) == 0) {
      _telemetry_metrics[i]->SetPeriod(msg->period);
      WS_DEBUG_PRINTLN("[telemetry] Updated existing metric's period");
      return true;
    }
  }

  TelemetryHardware *new_metric = new TelemetryHardware(msg->name, msg->period);

  // Gracefully ignore metrics this firmware doesn't know how to source, so a
  // single unsupported metric doesn't abort the rest of the checkin.
  if (new_metric->GetKind() == TELEMETRY_KIND_UNKNOWN) {
    WS_DEBUG_PRINTLN("[telemetry] WARNING: Unknown telemetry metric, ignoring");
    Ws.error_handler->publishComponentError(msg->name,
                                            "Unknown telemetry metric");
    delete new_metric;
    return true;
  }

  _telemetry_metrics.push_back(new_metric);

  WS_DEBUG_PRINT("[telemetry] New metric added! Period (s): ");
  WS_DEBUG_PRINTLNVAR(msg->period);
  return true;
}

/*!
    @brief  Handles a TelemetryRemove message from the broker. Stops
            reporting and destroys a telemetry metric instance.
    @param  msg
            The TelemetryRemove message.
    @return True if the metric was found and removed, False otherwise.
*/
bool TelemetryController::Handle_TelemetryRemove(ws_telemetry_Remove *msg) {
  WS_DEBUG_PRINT("[telemetry] Handle_TelemetryRemove: ");
  WS_DEBUG_PRINTLNVAR(msg->name);

  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    if (strcmp(_telemetry_metrics[i]->GetName(), msg->name) == 0) {
      delete _telemetry_metrics[i];
      _telemetry_metrics.erase(_telemetry_metrics.begin() + i);
      WS_DEBUG_PRINTLN("[telemetry] Removed!");
      return true;
    }
  }
  Ws.error_handler->publishComponentError(msg->name, "Telemetry metric not "
                                                     "found");
  return false;
}

/*!
    @brief  Encodes and publishes the telemetry model's current Event to the
            broker.
    @return True if the Event was encoded and published, False otherwise.
*/
bool TelemetryController::PublishTelemetry() {
  if (!_telemetry_model->EncodeEvent()) {
    WS_DEBUG_PRINTLN("[telemetry] ERROR: Failed to encode telemetry Event");
    return false;
  }
  if (!Ws.PublishD2b(ws_signal_DeviceToBroker_telemetry_tag,
                     _telemetry_model->GetD2B())) {
    WS_DEBUG_PRINTLN("[telemetry] ERROR: Failed to publish telemetry Event");
    return false;
  }
  return true;
}

/*!
    @brief  Update/polling loop for the telemetry controller.
    @param  force
            If true, forces a report on all metrics regardless of period.
*/
void TelemetryController::update(bool force) {
  // Bail out if there are no metrics to report
  if (_telemetry_metrics.empty())
    return;

  // When online we publish to the broker; when running offline we log the
  // reading to the SD card instead.
  bool is_offline = Ws._sdCardV2->isModeOffline();

  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    TelemetryHardware &metric = *(_telemetry_metrics[i]);

    // (force only) - Was metric previously reported this cycle?
    if (metric.did_read_send && force)
      continue;

    if (!metric.IsPeriodic()) {
      // Report-once metric (period 0) - report a single time, then mark
      // complete on every subsequent pass.
      if (metric.WasReported()) {
        metric.did_read_send = true;
        continue;
      }
    } else if (!force && !metric.IsTimerExpired()) {
      // Periodic metric whose timer hasn't expired yet
      continue;
    }

    // Read the metric and deliver it (publish online, or log to SD offline).
    bool delivered = false;
    switch (metric.GetKind()) {
    case TELEMETRY_KIND_RSSI: {
      float value = (float)metric.ReadRSSI();
      if (is_offline) {
        delivered = Ws._sdCardV2->LogTelemetryEventToSD(metric.GetName(), value,
                                                        ws_sensor_Type_T_RAW);
      } else {
        _telemetry_model->InitEventMsg(metric.GetName());
        _telemetry_model->SetValueFloat(ws_sensor_Type_T_RAW, value);
        delivered = PublishTelemetry();
      }
      break;
    }
    case TELEMETRY_KIND_BOOT_REASON: {
      const char *value = metric.ReadBootReason();
      if (is_offline) {
        delivered =
            Ws._sdCardV2->LogTelemetryEventToSD(metric.GetName(), value);
      } else {
        _telemetry_model->InitEventMsg(metric.GetName());
        _telemetry_model->SetValueString(ws_sensor_Type_T_BYTES, value);
        delivered = PublishTelemetry();
      }
      break;
    }
    default:
      // Shouldn't happen - unknown metrics are rejected at Handle_TelemetryAdd,
      // but log it here in case one ever slips through.
      WS_DEBUG_PRINT("[telemetry] WARNING: No reader for metric: ");
      WS_DEBUG_PRINTLNVAR(metric.GetName());
      metric.did_read_send = true;
      continue;
    }

    if (!delivered) {
      metric.did_read_send = false;
      continue;
    }

    metric.MarkPolled();
    if (!metric.IsPeriodic())
      metric.SetReported();
    metric.did_read_send = true;
  }
}

/*!
    @brief  Checks if all telemetry metrics have been reported.
    @return True if all metrics have been reported, False otherwise.
*/
bool TelemetryController::UpdateComplete() {
  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    if (!_telemetry_metrics[i]->did_read_send) {
      return false;
    }
  }
  return true;
}

/*!
    @brief  Resets all telemetry metrics' did_read_send flags to false.
*/
void TelemetryController::ResetFlags() {
  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    _telemetry_metrics[i]->did_read_send = false;
  }
}
