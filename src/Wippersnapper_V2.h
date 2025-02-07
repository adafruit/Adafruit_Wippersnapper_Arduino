/*!
 * @file Wippersnapper_V2.h
 *
 * This is the documentation for Adafruit's Wippersnapper firmware for the
 * Arduino platform. It is designed specifically to work with
 * Adafruit IO Wippersnapper IoT platform.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2024 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_V2_H
#define WIPPERSNAPPER_V2_H

// Debug Flags
// #DEBUG_PROFILE 1 ///< Enable debug output for function profiling

#define WS_DEBUG          ///< Define to enable debugging to serial terminal
#define WS_PRINTER Serial ///< Where debug messages will be printed

// Define actual debug output functions when necessary.
#ifdef WS_DEBUG
#define WS_DEBUG_PRINT(...)                                                    \
  { WS_PRINTER.print(__VA_ARGS__); } ///< Prints debug output.
#define WS_DEBUG_PRINTLN(...)                                                  \
  { WS_PRINTER.println(__VA_ARGS__); } ///< Prints line from debug output.
#define WS_DEBUG_PRINTHEX(...)                                                 \
  { WS_PRINTER.print(__VA_ARGS__, HEX); } ///< Prints debug output.
#else
#define WS_DEBUG_PRINT(...)                                                    \
  {} ///< Prints debug output
#define WS_DEBUG_PRINTLN(...)                                                  \
  {} ///< Prints line from debug output.
#endif

#define WS_DELAY_WITH_WDT(timeout)                                             \
  {                                                                            \
    unsigned long start = millis();                                            \
    while (millis() - start < timeout) {                                       \
      delay(10);                                                               \
      yield();                                                                 \
      feedWDT();                                                               \
      if (millis() < start) {                                                  \
        start = millis(); /* if rollover */                                    \
      }                                                                        \
    }                                                                          \
  } ///< Delay function

// Cpp STD
#include <functional>
#include <map>
#include <unordered_map>
#include <string>
#include <vector>

// Nanopb dependencies
#include <nanopb/pb_common.h>
#include <nanopb/pb_decode.h>
#include <nanopb/pb_encode.h>
#include <nanopb/ws_pb_helpers.h>
#include <pb.h>

// Include Signal Proto
#include "protos/checkin.pb.h"
#include "protos/digitalio.pb.h"
#include "protos/ds18x20.pb.h"
#include "protos/signal.pb.h"

// External libraries
#include "Adafruit_MQTT.h"      // MQTT Client
#include "Adafruit_SleepyDog.h" // Watchdog
#include "Arduino.h"            // Wiring
#include <SPI.h>                // SPI
#include <Wire.h>               // I2C

// Wippersnapper API Helpers
#include "Wippersnapper_Boards.h"
#include "components/statusLED/Wippersnapper_StatusLED.h"
#include "helpers/ws_helper_status.h"
#ifdef ARDUINO_ARCH_ESP32
#include "helpers/ws_helper_esp.h"
#endif

// Components (API v2)
#include "components/analogio/controller.h"
#include "components/checkin/model.h"
#include "components/digitalIO/controller.h"
#include "components/ds18x20/controller.h"
#include "components/i2c/controller.h"
#include "components/sensor/model.h"

// Display
#ifdef USE_DISPLAY
#include "display/ws_display_driver.h"
#include "display/ws_display_ui_helper.h"
#endif

#include "provisioning/ConfigJson.h"
#include "provisioning/sdcard/ws_sdcard.h"
#if defined(USE_TINYUSB)
#include "provisioning/tinyusb/Wippersnapper_FS.h"
#endif
#if defined(USE_LITTLEFS)
#include "provisioning/littlefs/WipperSnapper_LittleFS.h"
#endif

#define WS_VERSION                                                             \
  "1.0.0-alpha.1" ///< WipperSnapper app. version (semver-formatted)

#define WS_WDT_TIMEOUT 60000       ///< WDT timeout
#define WS_MAX_ALT_WIFI_NETWORKS 3 ///< Maximum number of alternative networks
/* MQTT Configuration */
#define WS_KEEPALIVE_INTERVAL_MS                                               \
  5000 ///< Session keepalive interval time, in milliseconds



// Forward declarations (API v1)
class Wippersnapper_FS;
class WipperSnapper_LittleFS;
class ws_sdcard;
#ifdef USE_DISPLAY
class ws_display_driver;
class ws_display_ui_helper;
#endif
// #ifdef ARDUINO_ARCH_ESP32
//class ws_ledc;
// #endif
class WipperSnapper_Component_I2C;
// class ws_servo;
// class ws_pwm;
// class ws_pixels;
// class ws_uart;

// Forward declarations (API v2)
class CheckinModel;
class SensorModel;
class DigitalIOController;
class AnalogIOController;
class DS18X20Controller;
class I2cController;

/**************************************************************************/
/*!
    @brief  Class that provides storage and functions for the Adafruit IO
            Wippersnapper interface.
*/
/**************************************************************************/
class Wippersnapper_V2 {
public:
  Wippersnapper_V2();
  virtual ~Wippersnapper_V2();

  void provision();

  // Global flags for the status led
  bool
      lockStatusNeoPixelV2; ///< True if status LED is using the status neopixel
  bool lockStatusDotStarV2; ///< True if status LED is using the status dotstar
  bool lockStatusLEDV2;     ///< True if status LED is using the built-in LED
  float status_pixel_brightnessV2 =
      STATUS_PIXEL_BRIGHTNESS_DEFAULT; ///< Global status pixel's brightness
                                       ///< (from 0.0 to 1.0)

  virtual void set_user_key();
  virtual void set_ssid_pass(const char *ssid, const char *ssidPassword);
  virtual void set_ssid_pass();
  virtual bool check_valid_ssid();

  virtual void _connect();
  virtual void _disconnect();
  void connect();
  void disconnect();

  virtual void getMacAddr();
  virtual int32_t getRSSI();
  virtual void setupMQTTClient(const char *clientID);

  virtual ws_status_t networkStatus();

  // Generators for device UID and MQTT topics
  bool generateDeviceUID();
  bool generateWSTopics();

  // High-level MQTT Publish
  bool PublishSignal(pb_size_t which_payload, void *payload);

  // Checkin API
  bool CreateCheckinRequest();
  void PollCheckinResponse();

  // run() loop
  ws_status_t run();
  void processPacketsV2();

  // Networking helpers
  void pingBrokerV2();
  void runNetFSMV2();

  // WDT helpers
  void enableWDTV2(int timeoutMS = 0);
  void feedWDTV2();

  // Error handling helpers
  void
  haltErrorV2(String error,
              ws_led_status_t ledStatusColor = WS_LED_STATUS_ERROR_RUNTIME);
  void errorWriteHangV2(String error);

  bool _is_offline_mode; ///< Global flag for if the device is in offline mode

  // TODO: Do we need this?
  ws_board_status_t _boardStatusV2 =
      WS_BOARD_DEF_IDLE; ///< Hardware's registration status

  // TODO: We really should look at making these static definitions, not dynamic
  // to free up space on the heap
  Wippersnapper_FS *_fileSystemV2;  ///< Instance of Filesystem (native USB)
  WipperSnapper_LittleFS
      *_littleFSV2;     ///< Instance of LittleFS Filesystem (non-native USB)
  ws_sdcard *_sdCardV2; ///< Instance of SD card class
#ifdef USE_DISPLAY
  ws_display_driver *_displayV2 = nullptr; ///< Instance of display driver class
  ws_display_ui_helper *_ui_helperV2 =
      nullptr; ///< Instance of display UI helper class
#endif
  // ws_pixels *_ws_pixelsComponentV2; ///< ptr to instance of ws_pixels class
  // ws_pwm *_pwmComponentV2;          ///< Instance of pwm class
  // ws_servo *_servoComponentV2;      ///< Instance of servo class
  // ws_uart *_uartComponentV2;        ///< Instance of UART class

  // API v2 Components
  CheckinModel *CheckInModel = nullptr; ///< Instance of CheckinModel class
  SensorModel *sensorModel = nullptr;   ///< Instance of SensorModel class
  DigitalIOController *digital_io_controller =
      nullptr; ///< Instance of DigitalIO controller class
  AnalogIOController *analogio_controller =
      nullptr; ///< Instance of AnalogIO controller
  DS18X20Controller *_ds18x20_controller =
      nullptr; ///< Instance of DS18X20 controller
  I2cController *_i2c_controller = nullptr;

  // TODO: does this really need to be global?
  uint8_t _macAddrV2[6];  /*!< Unique network iface identifier */
  char sUIDV2[13];        /*!< Unique network iface identifier */
  const char *_boardIdV2; /*!< Adafruit IO+ board string */
  Adafruit_MQTT *_mqttV2; /*!< Reference to Adafruit_MQTT, _mqtt. */

  // TODO: Audit this, does it need to be here?
  secretsConfig _configV2; /*!< Wippersnapper secrets.json as a struct. */
  networkConfig _multiNetworksV2[3]; /*!< Wippersnapper networks as structs. */
  bool _isWiFiMultiV2 = false; /*!< True if multiple networks are defined. */

  // TODO: Does this need to be within this class?
  int32_t totalDigitalPinsV2; /*!< Total number of digital-input capable pins */

  // TODO: Do these need to be here or can they sit within their function?
  char *throttleMessageV2; /*!< Pointer to throttle message data. */
  int throttleTimeV2;      /*!< Total amount of time to throttle the device, in
                            milliseconds. */

// enable LEDC if esp32
//#ifdef ARDUINO_ARCH_ESP32
  //ws_ledc *_ledcV2 = nullptr; ///< Pointer to LEDC object
//#endif
  bool got_checkin_response; ///< True if a checkin response was received, False
                             ///< otherwise.
  std::vector<std::vector<uint8_t>>
      _sharedConfigBuffers; ///< Shared JSON config buffers for offline mode
  JsonDocument _config_doc;
  uint8_t pin_sd_cs; ///< SD card chip select pin
private:
  void _initV2();

  // MQTT topics
  char *_topicB2d;
  char *_topicD2b;
  char *_topicError;
  char *_topicThrottle;

  // Adafruit_MQTT Subscription objects
  Adafruit_MQTT_Subscribe *_subscribeB2d;
  Adafruit_MQTT_Subscribe *_subscribeError;
  Adafruit_MQTT_Subscribe *_subscribeThrottle;

protected:
  ws_status_t _statusV2 = WS_IDLE; ///< Wippersnapper status

  uint32_t _last_mqtt_connectV2 = 0; /*!< Previous time when client connected to
                                          Adafruit IO, in milliseconds. */
  uint32_t _prv_pingV2 = 0; /*!< Previous time when client pinged Adafruit IO's
                             MQTT broker, in milliseconds. */
  uint32_t _prvKATBlinkV2 = 0; /*!< Previous time when client pinged Adafruit
                             IO's MQTT broker, in milliseconds. */

  // Device information
  const char *_deviceIdV2; /*!< Adafruit IO+ device identifier string */
  char *_device_uidV2;     /*!< Unique device identifier  */

};
extern Wippersnapper_V2 WsV2; ///< Global member variable for callbacks

#endif // WIPPERSNAPPER_V2_H
