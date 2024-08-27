/*!
 * @file model.cpp
 *
 * Model for the Wippersnapper checkin proto API.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "model.h"

/***********************************************************************/
/*!
    @brief  CheckinModel constructor
*/
/***********************************************************************/
CheckinModel::CheckinModel() {
  _CheckinRequest = wippersnapper_checkin_CheckinRequest_init_default;
  _CheckinResponse = wippersnapper_checkin_CheckinResponse_init_default;
}

/***********************************************************************/
/*!
    @brief  CheckinModel destructor
*/
/***********************************************************************/
CheckinModel::~CheckinModel() {
  _CheckinRequest = wippersnapper_checkin_CheckinRequest_init_default;
  _CheckinResponse = wippersnapper_checkin_CheckinResponse_init_default;
}

/***********************************************************************/
/*!
    @brief  Fills and creates a CheckinRequest message
    @param  hardware_uid
            Hardware's unique identifier.
    @param  firmware_version
            WipperSnapper firmware version.
*/
/***********************************************************************/
void CheckinModel::CreateCheckinRequest(const char *hardware_uid,
                                        const char *firmware_version) {
  strcpy(_CheckinRequest.hardware_uid, hardware_uid);
  strcpy(_CheckinRequest.firmware_version, firmware_version);
}

/***********************************************************************/
/*!
    @brief  Encodes a CheckinRequest message
    @returns True if the message was successfully encoded,
             False otherwise.
*/
/***********************************************************************/
bool CheckinModel::EncodeCheckinRequest() {
  // Obtain size of the CheckinRequest message
  size_t CheckinRequestSz;
  if (!pb_get_encoded_size(&CheckinRequestSz,
                           wippersnapper_checkin_CheckinRequest_fields,
                           &_CheckinRequest))
    return false;
  // Create a buffer for holding the CheckinRequest message
  uint8_t buf[CheckinRequestSz];

  // Create a stream that will write to buf
  pb_ostream_t msg_stream = pb_ostream_from_buffer(buf, sizeof(buf));
  // Encode the message
  return pb_encode(&msg_stream, wippersnapper_checkin_CheckinRequest_fields,
                   &_CheckinRequest);
}
