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
  strlcpy(dst.ssid, src["network_ssid"] | "unset-ssid", sizeof(dst.ssid));
  strlcpy(dst.pass, src["network_password"] | "", sizeof(dst.pass));
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

// Converts a displayConfigSPI structure to a JSON variant
void convertToJson(const displayConfigSPI &src, JsonVariant dst) {
  dst["pin_cs"] = src.pinCs;
  dst["pin_dc"] = src.pinDc;
  dst["pin_mosi"] = src.pinMosi;
  dst["pin_sck"] = src.pinSck;
  dst["pin_rst"] = src.pinRst;
}

// Extracts a displayConfigSPI structure from a JSON variant
void convertFromJson(JsonVariantConst src, displayConfigSPI &dst) {
  dst.pinCs = src["pin_cs"] | 40;
  dst.pinDc = src["pin_dc"] | 39;
  dst.pinMosi = src["pin_mosi"] | 0;
  dst.pinSck = src["pin_sck"] | 0;
  dst.pinRst = src["pin_rst"] | 41;
}

// Converts a displayConfig structure to a JSON variant
void convertToJson(const displayConfig &src, JsonVariant dst) {
  dst["driver"] = src.driver;
  dst["width"] = src.width;
  dst["height"] = src.height;
  dst["rotation"] = src.rotation;
  dst["spi_config"] = src.spiConfig;
}

// Extracts a displayConfig structure from a JSON variant
void convertFromJson(JsonVariantConst src, displayConfig &dst) {
  strlcpy(dst.driver, src["driver"] | "ST7789", sizeof(dst.driver));
  dst.width = src["width"] | 128;
  dst.height = src["height"] | 64;
  dst.rotation = src["rotation"] | 0;
  dst.spiConfig = src["spi_config"];
}
