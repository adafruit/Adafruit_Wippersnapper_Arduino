/*!
 * @file secretsConfig.h
 *
 * Contains user-defined structures for WipperSnapper JSON configuration files.
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
#ifndef CONFIG_H
#define CONFIG_H
struct networkConfig {
  char ssid[32];
  char pass[64];
};

struct secretsConfig {
  networkConfig network;
  char aio_url[64];
  char aio_user[31];
  char aio_key[33];
  uint16_t io_port;
  float status_pixel_brightness;
};
#endif // CONFIG_H