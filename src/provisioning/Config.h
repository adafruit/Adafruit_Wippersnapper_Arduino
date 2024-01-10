#ifndef CONFIG_H
#define CONFIG_H
struct networkConfig {
    char ssid[32];
    char pass[64];
};

struct mqttConfig {
    char server_url[64];
    char port[6];
    char aio_user[64];
    char aio_pass[30];
};

struct Config {
    networkConfig network;
    mqttConfig mqtt;
};
#endif // CONFIG_H