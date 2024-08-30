#ifndef WS_CHECKIN_MODEL_H
#define WS_CHECKIN_MODEL_H
#include "Wippersnapper_V2.h"

class CheckinModel {
public:
  CheckinModel();
  ~CheckinModel();
  void CreateCheckinRequest(const char *hardware_uid,
                            const char *firmware_version);
  bool EncodeCheckinRequest();
  bool DecodeCheckinResponse(pb_istream_t *stream);
  void ParseCheckinResponse();
  // TODO: Do we need this?
  wippersnapper_checkin_CheckinRequest GetCheckinRequest() {
    return _CheckinRequest;
  }

  void
  setCheckinResponse(wippersnapper_checkin_CheckinResponse_Response response);
  wippersnapper_checkin_CheckinResponse_Response getCheckinResponse();
  void setTotalGPIOPins(int32_t total_gpio_pins);
  int32_t getTotalGPIOPins();
  void setTotalAnalogPins(int32_t total_analog_pins);
  int32_t getTotalAnalogPins();
  void setReferenceVoltage(float reference_voltage);
  float getReferenceVoltage();

  // TODO: Shouldn't these be private?
  wippersnapper_checkin_CheckinRequest _CheckinRequest;
  wippersnapper_checkin_CheckinResponse _CheckinResponse;
  size_t CheckinRequestSz; // TODO: Do we need this?

private:
  wippersnapper_checkin_CheckinResponse_Response _checkin_response;
  int32_t _total_gpio_pins;
  int32_t _total_analog_pins;
  float _reference_voltage;
};

#endif // WS_CHECKIN_H