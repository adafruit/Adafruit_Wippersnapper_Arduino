#include "model.h"

CheckinModel::CheckinModel() {
  _CheckinRequest = wippersnapper_checkin_CheckinRequest_init_default;
  _CheckinResponse = wippersnapper_checkin_CheckinResponse_init_default;
}

CheckinModel::~CheckinModel() {
  _CheckinRequest = wippersnapper_checkin_CheckinRequest_init_default;
  _CheckinResponse = wippersnapper_checkin_CheckinResponse_init_default;
}

void CheckinModel::CreateCheckinRequest(const char *hardware_uid,
                                        const char *firmware_version) {
  strcpy(_CheckinRequest.hardware_uid, hardware_uid);
  strcpy(_CheckinRequest.firmware_version, firmware_version);
}

bool CheckinModel::EncodeCheckinRequest() {
  uint8_t buf[92]; // hardward_uid + firmware_version = 64 + 25 + 3 (overflow
                   // buffer space)= 92
  bool status_encode = false;
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  status_encode =
      pb_encode(&msg_stream, wippersnapper_checkin_CheckinRequest_fields,
                &_CheckinRequest);
  return status_encode;
}
