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
#include "Wippersnapper_V2.h"

class Wippersnapper_V2; ///< Forward declaration

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
  void ConfigureControllers();
  bool AddComponents();
  bool GotResponse();

private:
  ws_checkin_B2D _CheckinB2D = ws_checkin_B2D_init_zero; ///< Broker to Device message wrapper
  ws_checkin_D2B _CheckinD2B = ws_checkin_D2B_init_zero; ///< Device to Broker message wrapper
  bool _got_response;         ///< Flag indicating if response was received
};
extern Wippersnapper_V2 WsV2; ///< Wippersnapper V2 instance
#endif                        // WS_CHECKIN_H