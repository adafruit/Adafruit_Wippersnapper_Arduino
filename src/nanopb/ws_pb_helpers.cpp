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
#include "../Wippersnapper.h"

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
  if(!stream || !fields || !dest_struct) {
    WS_DEBUG_PRINTLN("Protobuf decode error: Invalid arguments to function");
    if (!stream) {
      WS_DEBUG_PRINTLN("stream is NULL");
    } else if (stream == nullptr) {
      WS_DEBUG_PRINTLN("stream is nullptr");
    }
    
    if (!fields) {
      WS_DEBUG_PRINTLN("fields is NULL");
    } else if (fields == nullptr) {
      WS_DEBUG_PRINTLN("fields is nullptr");
    }

    if (!dest_struct) {
      WS_DEBUG_PRINTLN("dest_struct is NULL");
    } else if (dest_struct == nullptr) {
      WS_DEBUG_PRINTLN("dest_struct is nullptr");
    }
    return false;
  }
  bool status = pb_decode(stream, fields, dest_struct);
  if (!status) {
    WS_DEBUG_PRINT("Protobuf decode error: ");
    WS_DEBUG_PRINTLN(PB_GET_ERROR(stream));
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
  if(!stream || !fields || !src_struct) {
    WS_DEBUG_PRINTLN("Protobuf encode error: Invalid arguments to function");
    if (!stream) {
      WS_DEBUG_PRINTLN("stream is NULL");
    } else if (stream == nullptr) {
      WS_DEBUG_PRINTLN("stream is nullptr");
    }
    
    if (!fields) {
      WS_DEBUG_PRINTLN("fields is NULL");
    } else if (fields == nullptr) {
      WS_DEBUG_PRINTLN("fields is nullptr");
    }

    if (!src_struct) {
      WS_DEBUG_PRINTLN("src_struct is NULL");
    } else if (src_struct == nullptr) {
      WS_DEBUG_PRINTLN("src_struct is nullptr");
    }
    return false;
  }
  bool status = pb_encode(stream, fields, src_struct);
  if (!status) {
    WS_DEBUG_PRINT("Protobuf encode error: ");
    WS_DEBUG_PRINTLN(PB_GET_ERROR(stream));
  }
  return status;
}