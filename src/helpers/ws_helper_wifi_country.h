/*!
 * @file ws_helper_wifi_country.h
 *
 * WiFi regulatory country codes shared by the ESP32 and ESP8266
 * platforms, so a country set in secrets.json behaves the same on both.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Tyeth Gundry 2026 for Adafruit Industries.
 *
 * MIT license, all text here must be included in any redistribution.
 *
 */
#ifndef WS_HELPER_WIFI_COUNTRY_H
#define WS_HELPER_WIFI_COUNTRY_H

#include "ws_helper_macros.h"
#include <Arduino.h>

/*!
@brief   Gets the 2.4GHz channel count for a WiFi country, starting at 1.
@note    The list mirrors the codes ESP-IDF's esp_wifi_set_country_code()
         supports (esp_wifi.h, "Supported country codes") in the pinned
         arduino-esp32 BSP, so recheck it when bumping the BSP. US, CA,
         MX and TW get 11 channels, JP gets 14 and the rest get 13. The
         ESP8266 SDK has no list of its own and takes the range as given.
@param   cc
         Uppercase ISO 3166 alpha-2 country code.
@returns The number of channels to use, 11, 13 or 14, or 0 if the code
         isn't supported.
*/
inline uint8_t ws_wifi_country_channels(const char *cc) {
  static const char ch13[] PROGMEM = "ATAUBEBGBRCHCNCYCZDEDKEEESFIFRGBGRHK"
                                     "HRHUIEINISITKRLILTLULVMTNLNONZPLPT"
                                     "ROSESISK";
  static const char ch11[] PROGMEM = "CAMXTWUS";
  if (cc == NULL || strlen(cc) != 2)
    return 0;
  if (strcmp(cc, "JP") == 0)
    return 14;
  for (size_t i = 0; i + 1 < sizeof(ch13); i += 2) {
    if (pgm_read_byte(&ch13[i]) == cc[0] &&
        pgm_read_byte(&ch13[i + 1]) == cc[1])
      return 13;
  }
  for (size_t i = 0; i + 1 < sizeof(ch11); i += 2) {
    if (pgm_read_byte(&ch11[i]) == cc[0] &&
        pgm_read_byte(&ch11[i + 1]) == cc[1])
      return 11;
  }
  return 0;
}

/*!
@brief   Prints the WiFi country in use and its channel range.
@param   cc
         Country code from the SDK, only the first two characters are
         printed as the third may be a ' ', 'O', 'I' or 'X' suffix.
@param   schan
         First channel.
@param   nchan
         Number of channels.
*/
inline void ws_print_wifi_country(const char *cc, uint8_t schan,
                                  uint8_t nchan) {
  char code[3] = {cc[0], cc[1], '\0'};
  WS_DEBUG_PRINT("WiFi country: ");
  WS_DEBUG_PRINTVAR(code);
  WS_DEBUG_PRINT(", channels ");
  WS_DEBUG_PRINTVAR(schan);
  WS_DEBUG_PRINT("-");
  WS_DEBUG_PRINTLNVAR(schan + nchan - 1);
}

#endif // WS_HELPER_WIFI_COUNTRY_H
