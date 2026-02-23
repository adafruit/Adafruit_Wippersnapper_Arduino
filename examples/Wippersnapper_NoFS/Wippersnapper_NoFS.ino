// Adafruit IO WipperSnapper
//
// This sketch is for devices which lack USB-MSD or LittleFS support such
// as the ESP32Dev for Wokwi Simulator
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2025
//
// All text above must be included in any redistribution.

#include "ws_platforms.h"
#define WS_DEBUG // Enable debug output
/************************ Adafruit IO Config *******************************/

// Visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME "YOUR_AIO_USERNAME"
#define IO_KEY "YOUR_AIO_KEY"
#define IO_URL "io.adafruit.com"
#define IO_PORT 8883
/**************************** WiFi Config ***********************************/
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"

ws_adapter_wifi wipper(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS, IO_URL, IO_PORT);

void setup() {
  // Provisioning must occur prior to serial init.
  wipper.provision();

  Serial.begin(115200);
  // while (!Serial) delay(10);

  wipper.connect();
}

void loop() { wipper.run(); }
