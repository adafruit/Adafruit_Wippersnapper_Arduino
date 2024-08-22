#include "model.h"

CheckinModel::CheckinModel() {
  CheckinRequest = wippersnapper_checkin_CheckinRequest_init_default;
  CheckinResponse = wippersnapper_checkin_CheckinResponse_init_default;
}

CheckinModel::~CheckinModel() {
  CheckinRequest = wippersnapper_checkin_CheckinRequest_init_default;
  CheckinResponse = wippersnapper_checkin_CheckinResponse_init_default;
}

void CheckinModel::CreateCheckinRequest(const char *hardware_uid,
                                        const char *firmware_version) {
  strcpy(CheckinRequest.hardware_uid, hardware_uid);
  strcpy(CheckinRequest.firmware_version, firmware_version);
}

bool CheckinModel::EncodeCheckinRequest() {
  uint8_t buf[92]; // hardward_uid + firmware_version = 64 + 25 + 3 (overflow
                   // buffer space)= 92
  bool status_encode = false;
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  status_encode =
      pb_encode(&msg_stream, wippersnapper_checkin_CheckinRequest_fields,
                &CheckinRequest);
  _len_checkin_request = msg_stream.bytes_written;
  return status_encode;
}

size_t CheckinModel::GetCheckinRequestSize() { return _len_checkin_request; }