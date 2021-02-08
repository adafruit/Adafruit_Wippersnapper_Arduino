/*!
 * @file Wippersnapper_Decoders.h
 *
 * This file provides protocol buffer decoders for the Wippersnapper
 * protocol API.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

// Nanopb dependencies
#include <pb.h>
#include <nanopb/pb_common.h>
#include <nanopb/pb_encode.h>
#include <nanopb/pb_decode.h>

// Message wrappers
#include <wippersnapper/description/v1/description.pb.h>    // description.proto
#include <wippersnapper/signal/v1/signal.pb.h>              // signal.proto
#include <wippersnapper/pin/v1/pin.pb.h>                    // pin.proto