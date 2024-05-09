#ifndef WS_PB_ENCODE_H
#define WS_PB_ENCODE_H

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include <Wippersnapper.h>

static bool ws_pb_decode(pb_istream_t *stream, const pb_msgdesc_t *fields,
                         void *dest_struct) {
  bool status = pb_decode(stream, fields, dest_struct);
  if (!status) {
    WS_DEBUG_PRINT("Protobuf decode error: ");
    WS_DEBUG_PRINTLN(PB_GET_ERROR(stream));
  }
  return status;
}

static bool ws_pb_encode(pb_ostream_t *stream, const pb_msgdesc_t *fields,
                         const void *src_struct) {
  bool status = pb_encode(stream, fields, src_struct);
  if (!status) {
    WS_DEBUG_PRINT("Protobuf encode error: ");
    WS_DEBUG_PRINTLN(PB_GET_ERROR(stream));
  }
  return status;
}

#endif // WS_PB_ENCODE_H