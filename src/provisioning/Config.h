#ifndef CONFIG_H
#define CONFIG_H
struct networkConfig {
    char ssid[32];
    char pass[64];
};

struct Config {
    networkConfig network;
    char aio_url[64];
    char aio_user[31];
    char aio_key[33];
    float status_pixel_brightness;
};
#endif // CONFIG_H