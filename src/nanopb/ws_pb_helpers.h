/*!
 * @file ws_pb_helpers.h
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
#ifndef WS_PB_ENCODE_H
#define WS_PB_ENCODE_H

#include "pb.h"

bool ws_pb_decode(pb_istream_t *stream, const pb_msgdesc_t *fields,
                         void *dest_struct);

bool ws_pb_encode(pb_ostream_t *stream, const pb_msgdesc_t *fields,
                         const void *src_struct);
                         
#endif // WS_PB_ENCODE_H