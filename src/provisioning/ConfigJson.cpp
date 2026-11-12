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
#include "Wippersnapper.h"

/*!
    @brief  Checks for an ASCII letter, without depending on the locale.
    @param  c
            The character to check.
    @returns True if c is A-Z or a-z.
*/
static bool isAsciiLetter(char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z');
}

/*!
    @brief  Parses the optional WiFi country, an ISO 3166 alpha-2 code.
    @note   Anything other than two letters is reported and ignored,
            rather than truncated into a different but valid code
            (e.g. "Germany" into "GE", Georgia).
    @param  src
            The network_country value from secrets.json, may be null.
    @param  dst
            Buffer of 3 for the uppercase code, "" if unset or invalid.
*/
static void parseCountryCode(JsonVariantConst src, char dst[3]) {
  dst[0] = '\0';
  if (src.isNull())
    return;
  const char *cc = src.is<const char *>() ? src.as<const char *>() : nullptr;
  if (cc != nullptr && cc[0] == '\0')
    return; // documented default, use the core's country
  if (cc != nullptr && strlen(cc) == 2 && isAsciiLetter(cc[0]) &&
      isAsciiLetter(cc[1])) {
    dst[0] = toupper(cc[0]);
    dst[1] = toupper(cc[1]);
    dst[2] = '\0';
    return;
  }
  WS_DEBUG_PRINT("WARNING: network_country in secrets.json must be a two "
                 "letter ISO 3166 code, such as \"GB\". Ignoring: ");
  WS_DEBUG_PRINTLNVAR(cc != nullptr ? cc : "(not a string)");
}

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
  if (strlen(src.network_country) > 0)
    dst["network_type_wifi"]["network_country"] = src.network_country;
  dst["status_pixel_brightness"] = src.status_pixel_brightness;
}

// Extracts a JSON file to a secretsConfig structure
void convertFromJson(JsonVariantConst src, secretsConfig &dst) {
  // Parse network credentials from secrets
  dst.network = src["network_type_wifi"];
  // Parse optional WiFi regulatory country, e.g. "GB", stored uppercase
  parseCountryCode(src["network_type_wifi"]["network_country"],
                   dst.network_country);
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