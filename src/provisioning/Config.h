#ifndef CONFIG_H
#define CONFIG_H
struct networkConfig {
    char ssid[32];
    char pass[64];
};

struct Config {
    networkConfig network;
    char server_url[64];
    char aio_user[64];
    char aio_pass[30];
    float status_pixel_brightness;
};
#endif // CONFIG_H