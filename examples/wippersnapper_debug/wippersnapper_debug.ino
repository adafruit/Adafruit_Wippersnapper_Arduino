// Adafruit IO WipperSnapper Beta (DEBUG BUILD ONLY!)
// Brent Rubell for Adafruit Industries, 2021 - 2023
#ifdef ARCH_ESP32
#include "esp_core_dump.h"
#endif

#include "Wippersnapper_Networking.h"
Wippersnapper_WiFi wipper;

// Enable debug output for beta builds
#define WS_DEBUG

void setup() {
  
#ifdef ARCH_ESP32
  // Configure the core dump to be saved to flash
  esp_err_t err = esp_core_dump_init(ESP_CORE_DUMP_FLASH); // Specify flash as storage
#endif

  
  // Provisioning must occur prior to serial init.
  wipper.provision();

  Serial.begin(115200); // wippersnapper serial
  Serial1.begin(115200);  // ESP-IDF messages serial
  Serial1.setDebugOutput(true); // Enable ESP-IDF messages over Serial1
  //while (!Serial) delay(10);

#ifdef ARCH_ESP32
  if (err != ESP_OK) {
    Serial.println("Core dump init failed!");
  }
#endif
  
  wipper.connect();
}

void loop() {
  wipper.run();
}
