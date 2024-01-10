#include "ConfigJson.h"

// Converts an ApConfig to JSON
// (automatically called by `aps.add(src.accessPoint[i])`)
void convertToJson(const networkConfig &src, JsonVariant dst) {
  dst["network_type_wifi"]["network_ssid"] = src.ssid;
  dst["network_type_wifi"]["network_password"] = src.pass;
}

// Extracts an ApConfig from JSON
// (automatically called by `dst.accessPoint[dst.accessPoints] = ap`)
void convertFromJson(JsonVariantConst src, networkConfig &dst) {
  strlcpy(dst.ssid, src["network_ssid"] | "testvar", sizeof(dst.ssid));
  strlcpy(dst.pass, src["network_password"] | "testvar", sizeof(dst.pass));
}

// Extracts a ServerConfig to JSON
// (automatically called by `dst["server"] = src.server`)
void convertToJson(const mqttConfig &src, JsonVariant dst) {
  dst["io_username"] = src.aio_user;
  dst["io_key"] = src.aio_pass;
  dst["url"] = src.server_url;
  dst["port"] = 8883;
}

// Convert a ServerConfig from JSON
// (automatically called by `dst.server = src["server"]`)
void convertFromJson(JsonVariantConst src, mqttConfig &dst) {
  strlcpy(dst.aio_user, src["io_username"] | "iousertest", sizeof(dst.aio_user));
  strlcpy(dst.aio_pass, src["io_key"] | "iokeytest", sizeof(dst.aio_pass));
  strlcpy(dst.server_url, src["io_url"] | "io.adafruit.com", sizeof(dst.server_url));
  strlcpy(dst.port, src["port"] | "8883", sizeof(dst.port));
}

// Converts a Config to JSON
// (automatically called by `config = doc.as<Config>()`)
void convertToJson(const Config &src, JsonVariant dst) {
  dst["network"] = src.network;
  dst["mqtt"] = src.mqtt;
}

// Extracts a Config from JSON
// (automatically called by `doc.set(config)`)
void convertFromJson(JsonVariantConst src, Config &dst) {
  dst.network = src["network_type_wifi"];
  dst.mqtt = src["mqtt"];
}
