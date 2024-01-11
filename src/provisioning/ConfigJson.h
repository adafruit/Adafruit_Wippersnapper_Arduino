/*!
 * @file ConfigJson.h
 *
 * Wippersnapper JSON Config File Converters
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
*/
#ifndef CONFIGJSON_H
#define CONFIGJSON_H
#define ARDUINOJSON_USE_DOUBLE 0
#define ARDUINOJSON_USE_LONG_LONG 1
#include <ArduinoJson.h>
#include "Config.h"

// Convert Config from/to JSON
void convertToJson(const Config &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, Config &dst);
#endif // CONFIGJSON_H