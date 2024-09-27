/*!
 * @file model.h
 *
 * Model for the Wippersnapper checkin proto API.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_CHECKIN_MODEL_H
#define WS_CHECKIN_MODEL_H
#include "Wippersnapper_V2.h"

/**************************************************************************/
/*!
    @brief  Provides an interface for creating, encoding, and parsing
            messages from checkin.proto.
*/
/**************************************************************************/
class CheckinModel {
public:
  CheckinModel();
  ~CheckinModel();
  // Request Message
  void CreateCheckinRequest(const char *hardware_uid,
                            const char *firmware_version);
  bool EncodeCheckinRequest();
  wippersnapper_checkin_CheckinRequest *getCheckinRequest() {
    return &_CheckinRequest;
  }
  // Response Message
  bool DecodeCheckinResponse(pb_istream_t *stream);
  void ParseCheckinResponse();
  void
  setCheckinResponse(wippersnapper_checkin_CheckinResponse_Response response);
  wippersnapper_checkin_CheckinResponse_Response getCheckinResponse();
  void setTotalGPIOPins(int32_t total_gpio_pins);
  int32_t getTotalGPIOPins();
  void setTotalAnalogPins(int32_t total_analog_pins);
  int32_t getTotalAnalogPins();
  void setReferenceVoltage(float reference_voltage);
  float getReferenceVoltage();

private:
  wippersnapper_checkin_CheckinRequest _CheckinRequest;
  wippersnapper_checkin_CheckinResponse _CheckinResponse;
  wippersnapper_checkin_CheckinResponse_Response _response;
  int32_t _total_gpio_pins;
  int32_t _total_analog_pins;
  float _reference_voltage;
};
#endif // WS_CHECKIN_H