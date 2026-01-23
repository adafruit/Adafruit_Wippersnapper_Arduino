/*!
 * @file wippersnapper.h
 *
 * This is the documentation for Adafruit's Wippersnapper firmware for the
 * Arduino platform. It is designed specifically to work with
 * Adafruit IO Wippersnapper IoT platform.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @copyright Copyright (c) Brent Rubell 2020-2026 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 */

#ifndef WIPPERSNAPPER_H
#define WIPPERSNAPPER_H

// Debug Flags
// #DEBUG_PROFILE 1 ///< Enable debug output for function profiling
// Debug Flags
#define WS_DEBUG          /**< Define to enable debugging to serial terminal */
#define WS_PRINTER Serial /**< Where debug messages will be printed */

/*!
    @brief  Debug print macros for WipperSnapper debugging output
    @details These macros provide debug output functionality when WS_DEBUG is
   defined
*/
#ifdef WS_DEBUG
#define WS_DEBUG_PRINT(...)                                                    \
  {                                                                            \
    WS_PRINTER.print(__VA_ARGS__);                                             \
  } /**< Print debug message to serial */
#define WS_DEBUG_PRINTLN(...)                                                  \
  {                                                                            \
    WS_PRINTER.println(__VA_ARGS__);                                           \
  } /**< Print debug message with newline                                      \
     */
#define WS_DEBUG_PRINTHEX(...)                                                 \
  {                                                                            \
    WS_PRINTER.print(__VA_ARGS__, HEX);                                        \
  } /**< Print debug message in hexadecimal */
#else
#define WS_DEBUG_PRINT(...)                                                    \
  {                                                                            \
  } /**< Debug print */
#define WS_DEBUG_PRINTLN(...)                                                  \
  {                                                                            \
  } /**< Debug println */
#endif

/*!
    @brief  delay() function for use with a watchdog timer
    @param  timeout
            Delay duration in milliseconds
*/
#define WS_DELAY_WITH_WDT(timeout)                                             \
  {                                                                            \
    unsigned long start = millis();                                            \
    while (millis() - start < timeout) {                                       \
      delay(10);                                                               \
      yield();                                                                 \
      Ws.FeedWDT();                                                            \
      if (millis() < start) {                                                  \
        start = millis();                                                      \
      }                                                                        \
    }                                                                          \
  }

// Cpp STD
#include <algorithm>
#include <functional>
#include <map>
#include <string>
#include <vector>

// Nanopb messages and dependencies
#include "protos/signal.pb.h"
#include <nanopb/pb_common.h>
#include <nanopb/pb_decode.h>
#include <nanopb/pb_encode.h>
#include <nanopb/ws_pb_helpers.h>

// External libraries
#include "Adafruit_MQTT.h"      // MQTT Client
#include "Adafruit_SleepyDog.h" // Watchdog
#include "Arduino.h"            // Wiring
#include <SPI.h>                // SPI
#include <Wire.h>               // I2C

// Wippersnapper API Helpers
#include "components/statusLED/Wippersnapper_StatusLED.h"
#include "helpers/ws_helper_status.h"
#include "ws_boards.h"
#ifdef ARDUINO_ARCH_ESP32
#include "helpers/ws_helper_esp.h"
#endif

// Components (API v2)
#include "components/analogIO/controller.h"
#include "components/checkin/model.h"
#include "components/digitalIO/controller.h"
#include "components/ds18x20/controller.h"
#include "components/error/controller.h"
#include "components/gps/controller.h"
#include "components/i2c/controller.h"
#include "components/pixels/controller.h"
#include "components/pwm/controller.h"
#include "components/sensor/model.h"
#include "components/servo/controller.h"
#include "components/sleep/controller.h"
#include "components/uart/controller.h"

#include "provisioning/ConfigJson.h"
#include "provisioning/sdcard/ws_sdcard.h"
#if defined(USE_TINYUSB)
#include "provisioning/tinyusb/Wippersnapper_FS.h"
#endif
#if defined(USE_LITTLEFS)
#include "provisioning/littlefs/WipperSnapper_LittleFS.h"
#endif

#define WS_VERSION                                                             \
  "2.0.0-beta.1" ///< WipperSnapper app. version (semver-formatted)

// Timeouts and intervals
#define WS_KEEPALIVE_INTERVAL_MS                                               \
  5000 ///< Session keepalive interval time, in milliseconds
#define WS_TIMEOUT_WDT 60000 ///< App WDT timeout, in milliseconds
#define WS_MQTT_POLL_TIMEOUT_MS                                                \
  10 ///< MQTT polling (processPackets()) timeout, in milliseconds
#define WS_DEFAULT_OFFLINE_HEARTBEAT_INTERVAL_MS                               \
  60000 ///< Default offline mode heartbeat interval, in milliseconds

#define WS_MAX_ALT_WIFI_NETWORKS 3 ///< Maximum number of alternative networks
#define WS_TOPIC_PREFIX_LEN 9      ///< (i.e: "/ws-d2b/")

// Forward declarations
class Wippersnapper_FS;
class WipperSnapper_LittleFS;
class ws_sdcard;
class CheckinModel;
class ErrorController;
class SensorModel;
class DigitalIOController;
class AnalogIOController;
class DS18X20Controller;
class GPSController;
class I2cController;
class PixelsController;
class PWMController;
class ServoController;
class UARTController;
class SleepController;

/*!
    @brief  Class that provides storage and functions for the Adafruit IO
            Wippersnapper interface.
*/
class wippersnapper {
public:
  wippersnapper();
  virtual ~wippersnapper();
  void provision();
  void run();

  // Global flags for the status led
  bool
      lockStatusNeoPixelV2; ///< True if status LED is using the status neopixel
  bool lockStatusDotStarV2; ///< True if status LED is using the status dotstar
  bool lockStatusLEDV2;     ///< True if status LED is using the built-in LED
  float status_pixel_brightnessV2 =
      STATUS_PIXEL_BRIGHTNESS_DEFAULT; ///< Global status pixel's brightness
                                       ///< (from 0.0 to 1.0)

  // Network interface virtual functions
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
  bool PublishD2b(pb_size_t which_payload, void *payload);

  // run() loop
  void ProcessPackets();

  // Networking helpers
  void pingBrokerV2();
  void NetworkFSM(bool initial_connect = false);

  // WDT helpers
  int EnableWDT(int timeout_ms = 0);
  int ReconfigureWDT(int timeout_ms);
  void FeedWDT();
  void BlinkKATStatus();

  // Error handling helpers
  void haltErrorV2(const char *error,
                   ws_led_status_t ledStatusColor = WS_LED_STATUS_ERROR_RUNTIME,
                   bool reboot = true);
  void errorWriteHangV2(const char *error);

  bool _is_offline_mode; ///< Global flag for if the device is in offline mode

  // TODO: Do we need this?
  ws_board_status_t _boardStatusV2 =
      WS_BOARD_DEF_IDLE; ///< Hardware's registration status

  // TODO: We really should look at making these static definitions, not dynamic
  // to free up space on the heap
  Wippersnapper_FS *_fileSystemV2; ///< Instance of Filesystem (native USB)
  WipperSnapper_LittleFS
      *_littleFSV2;     ///< Instance of LittleFS Filesystem (non-native USB)
  ws_sdcard *_sdCardV2; ///< Instance of SD card class

  // API v2 Components
  CheckinModel *CheckInModel = nullptr; ///< Instance of CheckinModel class
  ErrorController *error_controller =
      nullptr;                         ///< Instance of ErrorController class
  SensorModel *sensor_model = nullptr; ///< Instance of SensorModel class
  DigitalIOController *digital_io_controller =
      nullptr; ///< Instance of DigitalIO controller class
  AnalogIOController *analogio_controller =
      nullptr; ///< Instance of AnalogIO controller
  DS18X20Controller *_ds18x20_controller =
      nullptr;                              ///< Instance of DS18X20 controller
  GPSController *_gps_controller = nullptr; ///< Instance of GPS controller
  I2cController *_i2c_controller = nullptr; ///< Instance of I2C controller
  PixelsController *_pixels_controller =
      nullptr;                              ///< Instance of Pixels controller
  PWMController *_pwm_controller = nullptr; ///< Instance of PWM controller
  ServoController *_servo_controller =
      nullptr;                                ///< Instance of Servo controller
  UARTController *_uart_controller = nullptr; ///< Instance of UART controller
#ifdef ARDUINO_ARCH_ESP32
  SleepController *_sleep_controller =
      nullptr; ///< Instance of sleep controller
#endif

  // TODO: does this really need to be global?
  uint8_t _macAddrV2[6];  /*!< Unique network iface identifier */
  char sUIDV2[13];        /*!< Unique hardware identifier */
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

  std::vector<std::vector<uint8_t>>
      _sharedConfigBuffers; ///< Shared JSON config buffers for offline mode
  JsonDocument _config_doc; ///< Storage for the config.json file
  uint8_t pin_sd_cs;        ///< SD card chip select pin
private:
  // Separate loop() functions, depending on power mode
  void loop();
#ifdef ARDUINO_ARCH_ESP32
  void loopSleep();
  void ResetAllControllerFlags();
#endif
  void blinkOfflineHeartbeat();

  // MQTT topics
  char *_topicB2d;
  char *_topicD2b;

  // Adafruit_MQTT Subscription objects
  Adafruit_MQTT_Subscribe *_subscribeB2d =
      nullptr; ///< MQTT subscription object for B2D topic

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
extern wippersnapper Ws; ///< Global member variable for callbacks

#endif // WIPPERSNAPPER_H
