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
    @brief  If a metric of the given type already exists, updates its period.
    @param  type
            The telemetry metric type.
    @param  period
            The new reporting interval, in seconds.
    @return True if an existing metric was found and updated, False otherwise.
*/
bool TelemetryController::updatePeriod(ws_telemetry_Type type, float period) {
  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    if (_telemetry_metrics[i]->GetType() == type) {
      _telemetry_metrics[i]->SetPeriod(period);
      WS_DEBUG_PRINTLN("[telemetry] Updated existing metric's period");
      if (!_telemetry_metrics[i]->IsPeriodic()) {
        // Reset the reported state to allow a report-once metric to published
        _telemetry_metrics[i]->SetReported(false);
      }
      return true;
    }
  }
  return false;
}

/*!
    @brief  Handles a TelemetryAdd message from the broker. Creates (or
            updates) a telemetry metric instance and starts reporting it.
    @param  msg
            The TelemetryAdd message.
    @return True if the metric was successfully registered (or an unsupported
            metric type was gracefully ignored), False otherwise.
*/
bool TelemetryController::Handle_TelemetryAdd(ws_telemetry_Add *msg) {
  WS_DEBUG_PRINT("[telemetry] Handle_TelemetryAdd: ");
  WS_DEBUG_PRINTLNVAR(TelemetryTypeName(msg->type));

  // Gracefully ignore metrics this firmware doesn't know how to source, so a
  // single unsupported metric doesn't abort the rest of the checkin. Checked
  // before allocating the instance.
  if (!TelemetryTypeIsSupported(msg->type)) {
    Ws->error_handler->publishComponentError(TelemetryTypeName(msg->type),
                                            "Unsupported telemetry metric");
    return true;
  }

  // If a metric of this type already exists, update its period instead of
  // registering a duplicate.
  if (updatePeriod(msg->type, msg->period))
    return true;

  TelemetryHardware *new_metric = new TelemetryHardware(msg->type, msg->period);
  // Confirm the instance was allocated and constructed for the requested
  // metric before tracking it.
  if (new_metric == nullptr || new_metric->GetType() != msg->type) {
    Ws->error_handler->publishComponentError(
        TelemetryTypeName(msg->type), "Failed to create telemetry metric");
    delete new_metric;
    return false;
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
  WS_DEBUG_PRINTLNVAR(TelemetryTypeName(msg->type));

  for (size_t i = 0; i < _telemetry_metrics.size(); i++) {
    if (_telemetry_metrics[i]->GetType() == msg->type) {
      delete _telemetry_metrics[i];
      _telemetry_metrics.erase(_telemetry_metrics.begin() + i);
      WS_DEBUG_PRINTLN("[telemetry] Removed!");
      return true;
    }
  }
  Ws->error_handler->publishComponentError(TelemetryTypeName(msg->type),
                                          "Telemetry metric not found");
  return false;
}

/*!
    @brief  Encodes the telemetry model's current Event and publishes it to
            the broker.
    @return True if the Event was encoded and published, False otherwise.
*/
bool TelemetryController::encodeAndPublish() {
  if (!_telemetry_model->EncodeEvent()) {
    WS_DEBUG_PRINTLN("[telemetry] ERROR: Failed to encode telemetry Event");
    return false;
  }
  if (!Ws->PublishD2b(ws_signal_DeviceToBroker_telemetry_tag,
                     _telemetry_model->GetD2B())) {
    WS_DEBUG_PRINTLN("[telemetry] ERROR: Failed to publish telemetry Event");
    return false;
  }
  return true;
}

/*!
    @brief  Fills the telemetry model with a float reading, then encodes and
            publishes the Event to the broker.
    @param  type
            The telemetry metric type.
    @param  value_type
            The reading's SensorType.
    @param  value
            The reading to report.
    @return True if the Event was encoded and published, False otherwise.
*/
bool TelemetryController::Publish(ws_telemetry_Type type,
                                  ws_sensor_Type value_type, float value) {
  _telemetry_model->InitEventMsg(type);
  _telemetry_model->SetValueFloat(value_type, value);
  return encodeAndPublish();
}

/*!
    @brief  Fills the telemetry model with a string reading, then encodes and
            publishes the Event to the broker.
    @param  type
            The telemetry metric type.
    @param  value_type
            The reading's SensorType.
    @param  value
            The reading to report.
    @return True if the Event was encoded and published, False otherwise.
*/
bool TelemetryController::Publish(ws_telemetry_Type type,
                                  ws_sensor_Type value_type,
                                  const char *value) {
  _telemetry_model->InitEventMsg(type);
  _telemetry_model->SetValueString(value_type, value);
  return encodeAndPublish();
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

    // Read the metric and deliver it: when online we publish to the broker;
    // when running offline we log the reading to the SD card instead.
    bool delivered = false;
    switch (metric.GetType()) {
    case ws_telemetry_Type_TM_RSSI: {
      float value = (float)metric.ReadRSSI();
      if (Ws->_sdCardV2->isModeOffline()) {
        delivered = Ws->_sdCardV2->LogTelemetryEventToSD(metric.GetName(), value,
                                                        ws_sensor_Type_T_RAW);
      } else {
        delivered = Publish(metric.GetType(), ws_sensor_Type_T_RAW, value);
      }
      break;
    }
    case ws_telemetry_Type_TM_BOOT_REASON: {
      const char *value = metric.ReadBootReason();
      if (Ws->_sdCardV2->isModeOffline()) {
        delivered =
            Ws->_sdCardV2->LogTelemetryEventToSD(metric.GetName(), value);
      } else {
        delivered = Publish(metric.GetType(), ws_sensor_Type_T_BYTES, value);
      }
      break;
    }
    default:
      // Shouldn't happen - unsupported metrics are rejected at
      // Handle_TelemetryAdd, but log it here in case one ever slips through.
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
