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