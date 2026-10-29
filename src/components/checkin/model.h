/*!
 * @file src/components/checkin/model.h
 *
 * Model for the Wippersnapper checkin proto API.
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
#ifndef WS_CHECKIN_MODEL_H
#define WS_CHECKIN_MODEL_H
#include "wippersnapper.h"

#define CHECKIN_SHA_LEN 32 ///< SHA-256 digest length of a checkin response

class wippersnapper; ///< Forward declaration

/*!
    @brief  How to apply a component Add from a checkin response.
*/
enum CheckinAction {
  CHECKIN_ADD,     ///< New component, add it
  CHECKIN_REPLACE, ///< Existing component changed, replace it
  CHECKIN_SKIP     ///< Existing component unchanged, leave it alone
};

/*!
    @brief  A component applied from a checkin response, kept to reconcile
            later checkin responses against.
*/
struct CheckinComponent {
  pb_size_t type;                   ///< BrokerToDevice payload tag
  std::vector<uint8_t> remove;      ///< Encoded Remove message identifying it
  uint8_t add_sha[CHECKIN_SHA_LEN]; ///< SHA-256 of its raw Add message
  bool seen;                        ///< Present in the response being handled
};

/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from checkin.proto.
*/
class CheckinModel {
public:
  CheckinModel();
  ~CheckinModel();
  bool Checkin(const char *hardware_uid, const char *firmware_version);
  bool ProcessResponse(pb_istream_t *stream);
  bool ValidateResponse();
  void ConfigureControllers();
  bool GotResponse();
  bool ResponseChanged(pb_istream_t *stream);
  void SaveResponseHash();
  void RemoveStaleComponents(pb_istream_t *stream);
  bool Complete();
  void configureSleep();
  int32_t GetTotalGPIOPins();
  int32_t GetTotalAnalogPins();
  float GetReferenceVoltage();

private:
  static bool
  cbSetupResponse(pb_istream_t *stream, const pb_field_t *field,
                  void **arg); //< Pre-decode callback to set up component_adds
  static bool cbDigitalAdds(pb_istream_t *stream, const pb_field_t *field,
                            void **arg);
  static bool cbAnalogAdds(pb_istream_t *stream, const pb_field_t *field,
                           void **arg);
  static bool cbServoAdds(pb_istream_t *stream, const pb_field_t *field,
                          void **arg);
  static bool cbPWMAdds(pb_istream_t *stream, const pb_field_t *field,
                        void **arg);
  static bool cbPixelsAdds(pb_istream_t *stream, const pb_field_t *field,
                           void **arg);
  static bool cbDs18x20Adds(pb_istream_t *stream, const pb_field_t *field,
                            void **arg);
  static bool cbUartAdds(pb_istream_t *stream, const pb_field_t *field,
                         void **arg);
  static bool cbI2cAdds(pb_istream_t *stream, const pb_field_t *field,
                        void **arg);
  static bool cbTelemetryAdds(pb_istream_t *stream, const pb_field_t *field,
                              void **arg);
  CheckinAction ReconcileAdd(const pb_istream_t *raw, pb_size_t type,
                             const pb_msgdesc_t *remove_fields,
                             const void *remove_msg);
  bool CommitAdd(bool added);
  bool IsSleepEnabled();
  ws_sleep_SleepConfig *GetSleepConfig();
  ws_checkin_B2D _CheckinB2D =
      ws_checkin_B2D_init_zero; ///< Broker to Device message wrapper
  ws_checkin_D2B _CheckinD2B =
      ws_checkin_D2B_init_zero; ///< Device to Broker message wrapper
  bool _got_response;           ///< Flag indicating if response was received
  uint8_t _response_sha[CHECKIN_SHA_LEN]; ///< SHA-256 of this response
  bool _has_response_sha = false; ///< True if _response_sha was computed
  CheckinComponent _pending;      ///< Component Add being applied
  int _pending_idx = -1;          ///< Index of _pending's existing entry, or -1
  bool _has_pending = false;      ///< True if _pending awaits CommitAdd()
  bool _mark_only = false;        ///< True while RemoveStaleComponents() marks
};
extern wippersnapper *Ws; ///< Wippersnapper V2 instance
#endif                    // WS_CHECKIN_H