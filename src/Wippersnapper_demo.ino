// Adafruit IO WipperSnapper Beta
//
//
// NOTE: This software is a BETA release and in active development.
// Please report bugs or errors to https://github.com/adafruit/Adafruit_Wippersnapper_Arduino/issues
//
//
// Adafruit invests time and resources providing this open source code.
// Please support Adafruit and open source hardware by purchasing
// products from Adafruit!
//
// Brent Rubell for Adafruit Industries, 2021-2022
//
// All text above must be included in any redistribution.
// #include <GDBStub.h> // Include GDB Stub for debugging esp8266
#include "Wippersnapper_Networking.h"
Wippersnapper_WiFi wipper;

// Enable debug output for beta builds
#define WS_DEBUG

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn off the built-in LED
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);   // Turn on the built-in LED
  // Provisioning must occur prior to serial init.
  wipper.provision();
  digitalWrite(LED_BUILTIN, LOW);   // Turn on the built-in LED
  Serial.begin(115200);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn off the built-in LED
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);   // Turn on the built-in LED
  WS_DEBUG_PRINTLN("WipperSnapper Demo Starting...");
#ifdef ARDUINO_ARCH_ESP8266
  WS_DEBUG_PRINT("Boot heap: ");
  WS_DEBUG_PRINTLN(ESP.getFreeHeap());
  WS_DEBUG_PRINT("Boot max free block: ");
  WS_DEBUG_PRINTLN(ESP.getMaxFreeBlockSize());
  WS_DEBUG_PRINT("Boot heap fragmentation (%): ");
  WS_DEBUG_PRINTLN(ESP.getHeapFragmentation());
#endif
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn off the built-in LED
  // while (!Serial) delay(10);
  // gdbstub_init();

  wipper.connect();
  digitalWrite(LED_BUILTIN, LOW);   // Turn on the built-in LED

}

void loop() {
  wipper.run();
}