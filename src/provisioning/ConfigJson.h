/*!
 * @file ConfigJson.h
 *
 * Wippersnapper JSON secretsConfig File Converters
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
#include "Config.h"
#include <ArduinoJson.h>

// Convert secretsConfig from/to JSON
void convertToJson(const secretsConfig &src, JsonVariant dst);
void convertFromJson(JsonVariantConst src, secretsConfig &dst);
#endif // CONFIGJSON_H