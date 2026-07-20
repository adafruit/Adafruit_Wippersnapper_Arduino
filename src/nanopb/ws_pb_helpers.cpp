/*!
 * @file ws_pb_helpers.cpp
 *
 * Protobuf encode/decode helpers with error logging for Wippersnapper.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "ws_pb_helpers.h"
#include "../wippersnapper.h"

// *****************************************************************************
/*!
    @brief    Decodes a protobuf message from a stream and prints any error.
    @param    stream
              The stream to decode from.
    @param    fields
              The protobuf message fields.
    @param    dest_struct
              The destination struct to decode into.
    @return   True if decode was successful, false otherwise.
!*/
// *****************************************************************************
bool ws_pb_decode(pb_istream_t *stream, const pb_msgdesc_t *fields,
                  void *dest_struct) {
  bool status = pb_decode(stream, fields, dest_struct);
  if (!status) {
    WS_DEBUG_PRINT("Protobuf decode error: ");
    WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(stream));
  }
  return status;
}

// *****************************************************************************
/*!
    @brief    Encodes a protobuf message to a stream and prints any error.
    @param    stream
              The stream to encode to.
    @param    fields
              The protobuf message fields.
    @param    src_struct
              The source struct to encode from.
    @return   True if encode was successful, false otherwise.
!*/
// *****************************************************************************
bool ws_pb_encode(pb_ostream_t *stream, const pb_msgdesc_t *fields,
                  const void *src_struct) {
  bool status = pb_encode(stream, fields, src_struct);
  if (!status) {
    WS_DEBUG_PRINT("Protobuf encode error: ");
    WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(stream));
  }
  return status;
}

// *****************************************************************************
/*!
    @brief    Decodes a submessage that lives inside a `oneof` from an envelope
              stream WITHOUT relying on decode callbacks pre-set on the oneof
              union member.
    @details  nanopb zeroes a oneof member the first time it sets the union
              tag, which wipes any callbacks configured on that member before
              decoding the envelope (so repeated/callback fields silently
              decode as empty). This helper instead walks the envelope stream,
              finds the requested field, and decodes it as a standalone message,
              where callbacks configured on `dest_struct` survive.
    @param    stream
              The envelope input stream (e.g. a component B2D).
    @param    field_tag
              Protobuf field number of the oneof submessage to decode.
    @param    submsg_fields
              nanopb message descriptor for the submessage.
    @param    dest_struct
              Destination struct; may have decode callbacks pre-configured.
    @return   True if the field was found and decoded, false otherwise.
!*/
// *****************************************************************************
bool ws_pb_decode_oneof_submsg(pb_istream_t *stream, uint32_t field_tag,
                               const pb_msgdesc_t *submsg_fields,
                               void *dest_struct) {
  pb_wire_type_t wire_type;
  uint32_t tag;
  bool eof = false;
  bool decoded = false;
  while (pb_decode_tag(stream, &wire_type, &tag, &eof)) {
    if (eof)
      break;
    if (tag == field_tag && wire_type == PB_WT_STRING) {
      pb_istream_t substream;
      if (!pb_make_string_substream(stream, &substream))
        return false;
      bool ok = pb_decode(&substream, submsg_fields, dest_struct);
      if (!pb_close_string_substream(stream, &substream) || !ok) {
        if (!ok) {
          WS_DEBUG_PRINT("Protobuf decode error: ");
          WS_DEBUG_PRINTLNVAR(PB_GET_ERROR(&substream));
        }
        return false;
      }
      decoded = true;
    } else if (!pb_skip_field(stream, wire_type)) {
      return false;
    }
  }
  return decoded;
}