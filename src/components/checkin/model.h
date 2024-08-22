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
  size_t GetCheckinRequestSize();
  // TODO: Handle the Checkin Response
  wippersnapper_checkin_CheckinRequest CheckinRequest;
  wippersnapper_checkin_CheckinResponse CheckinResponse;

private:
  size_t _len_checkin_request;
};

#endif // WS_CHECKIN_H