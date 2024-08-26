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
  wippersnapper_checkin_CheckinRequest GetCheckinRequest() {
    return _CheckinRequest;
  }
  // TODO: Handle the Checkin Response
  wippersnapper_checkin_CheckinRequest _CheckinRequest;
  wippersnapper_checkin_CheckinResponse _CheckinResponse;
  size_t CheckinRequestSz;
private:

};

#endif // WS_CHECKIN_H