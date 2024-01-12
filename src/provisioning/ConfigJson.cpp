/*!
 * @file ConfigJson.cpp
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
#include "ConfigJson.h"

// Converts a network configuration structure to a JSON variant
void convertToJson(const networkConfig &src, JsonVariant dst) {
  dst["network_ssid"] = src.ssid;
  dst["network_password"] = src.pass;
}

// Extracts a network configuration structure from a JSON variant
void convertFromJson(JsonVariantConst src, networkConfig &dst) {
  strlcpy(dst.ssid, src["network_ssid"] | "testvar", sizeof(dst.ssid));
  strlcpy(dst.pass, src["network_password"] | "testvar", sizeof(dst.pass));
}

// Converts a secretsConfig structure to a JSON variant
void convertToJson(const secretsConfig &src, JsonVariant dst) {
  dst["io_username"] = src.aio_user;
  dst["io_key"] = src.aio_key;
  dst["network_type_wifi"] = src.network;
  dst["status_pixel_brightness"] = src.status_pixel_brightness;
}

// Extracts a JSON file to a secretsConfig structure
void convertFromJson(JsonVariantConst src, secretsConfig &dst) {
  // Parse network credentials from secrets
  dst.network = src["network_type_wifi"];
  // Parse IO credentials from secrets
  strlcpy(dst.aio_user, src["io_username"] | "YOUR_IO_USERNAME_HERE",
          sizeof(dst.aio_user));
  strlcpy(dst.aio_key, src["io_key"] | "YOUR_IO_KEY_HERE", sizeof(dst.aio_key));
  strlcpy(dst.aio_url, src["io_url"] | "io.adafruit.com", sizeof(dst.aio_url));
  // Parse status pixel brightness from secrets
  dst.status_pixel_brightness = src["status_pixel_brightness"] | 0.2;
  // Parse MQTT port from secrets, if exists
  dst.io_port = src["io_port"] | 8883;
}
