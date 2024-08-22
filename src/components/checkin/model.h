#ifndef WS_CHECKIN_MODEL_H
#define WS_CHECKIN_MODEL_H
// Nanopb dependencies
#include "../../protos/checkin.pb.h"
#include <nanopb/pb_common.h>
#include <nanopb/pb_decode.h>
#include <nanopb/pb_encode.h>
#include <pb.h>

class CheckinModel {
public:
  CheckinModel();
  ~CheckinModel();
  void CreateCheckinRequest(const char *hardware_uid,
                            const char *firmware_version);
  bool EncodeCheckinRequest();
  // TODO: Handle the Checkin Response
  wippersnapper_checkin_CheckinRequest CheckinRequest;
  wippersnapper_checkin_CheckinResponse CheckinResponse;

private:
  size_t _len_checkin_request;
};

#endif // WS_CHECKIN_H