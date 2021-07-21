/*!
 * @file Wippersnapper.h
 *
 * This is the documentation for Adafruit's Wippersnapper firmware for the
 * Arduino platform. It is designed specifically to work with
 * Adafruit IO Wippersnapper IoT platform.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_H
#define WIPPERSNAPPER_H

#include <queue>
#include <vector>

// Nanopb dependencies
#include <nanopb/pb_common.h>
#include <nanopb/pb_decode.h>
#include <nanopb/pb_encode.h>
#include <pb.h>

#include <wippersnapper/description/v1/description.pb.h> // description.proto
#include <wippersnapper/pin/v1/pin.pb.h>                 // pin.proto
#include <wippersnapper/signal/v1/signal.pb.h>           // signal.proto

// Wippersnapper API Helpers
#include "Wippersnapper_Boards.h"
#include "Wippersnapper_Registration.h"
#include "Wippersnapper_StatusLED_Colors.h"

// Wippersnapper GPIO
#include "Wippersnapper_AnalogIO.h"
#include "Wippersnapper_DigitalGPIO.h"

// WipperSnapper I2C Component
#include "components/WipperSnapper_Component_I2C.h"

// External libraries
#include "Adafruit_MQTT.h" // MQTT Client
#include "Arduino.h"       // Wiring

#if defined(ADAFRUIT_METRO_M4_AIRLIFT_LITE) || ADAFRUIT_PYPORTAL
#include <Adafruit_SleepyDog.h>
#else // ESP32/ESP32-S2
#include <esp_task_wdt.h>
#endif

// Note: These might be better off in their respective wrappers
#include <Adafruit_DotStar.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <SPI.h>

// Uncomment for staging builds
#define USE_STAGING

#ifdef USE_STAGING
#define IO_MQTT_SERVER "io.adafruit.us"
#else
#define IO_MQTT_SERVER "io.adafruit.com"
#endif

#ifdef USE_TINYUSB
#include "provisioning/tinyusb/Wippersnapper_FS.h"
#endif

#ifdef USE_NVS
#include "provisioning/Wippersnapper_ESP32_nvs.h"
#endif

// Library version (semver-formatted)
#define WS_VERSION "1.0.0-beta.3"

// Reserved Adafruit IO MQTT topics
#define TOPIC_IO_THROTTLE "/throttle" ///< Adafruit IO Throttle MQTT Topic
#define TOPIC_IO_ERRORS "/errors"     ///< Adafruit IO Error MQTT Topic

// Reserved Wippersnapper topics
#define TOPIC_WS "/wprsnpr/"       ///< WipperSnapper topic
#define TOPIC_DESCRIPTION "/info/" ///< Registration sub-topic
#define TOPIC_SIGNALS "/signals/"  ///< Signals sub-topic
#define TOPIC_I2C "/i2c/"          ///< I2C sub-topic

#define WS_DEBUG          ///< Define to enable debugging to serial terminal
#define WS_PRINTER Serial ///< Where debug messages will be printed

// Define actual debug output functions when necessary.
#ifdef WS_DEBUG
#define WS_DEBUG_PRINT(...)                                                    \
  { WS_PRINTER.print(__VA_ARGS__); } ///< Prints debug output.
#define WS_DEBUG_PRINTLN(...)                                                  \
  { WS_PRINTER.println(__VA_ARGS__); } ///< Prints line from debug output.
#else
#define WS_DEBUG_PRINT(...)                                                    \
  {} ///< Prints debug output
#define WS_DEBUG_PRINTLN(...)                                                  \
  {} ///< Prints line from debug output.
#endif

/** Defines the Adafruit IO connection status */
typedef enum {
  WS_IDLE = 0,               // Waiting for connection establishement
  WS_NET_DISCONNECTED = 1,   // Network disconnected
  WS_DISCONNECTED = 2,       // Disconnected from Adafruit IO
  WS_FINGERPRINT_UNKOWN = 3, // Unknown WS_SSL_FINGERPRINT

  WS_NET_CONNECT_FAILED = 10,  // Failed to connect to network
  WS_CONNECT_FAILED = 11,      // Failed to connect to Adafruit IO
  WS_FINGERPRINT_INVALID = 12, // Unknown WS_SSL_FINGERPRINT
  WS_AUTH_FAILED = 13, // Invalid Adafruit IO login credentials provided.
  WS_SSID_INVALID =
      14, // SSID is "" or otherwise invalid, connection not attempted

  WS_NET_CONNECTED = 20,           // Connected to Adafruit IO
  WS_CONNECTED = 21,               // Connected to network
  WS_CONNECTED_INSECURE = 22,      // Insecurely (non-SSL) connected to network
  WS_FINGERPRINT_UNSUPPORTED = 23, // Unsupported WS_SSL_FINGERPRINT
  WS_FINGERPRINT_VALID = 24,       // Valid WS_SSL_FINGERPRINT
  WS_BOARD_DESC_INVALID = 25,      // Unable to send board description
  WS_BOARD_RESYNC_FAILED = 26      // Board sync failure
} ws_status_t;

/** Defines the Adafruit IO MQTT broker's connection return codes */
typedef enum {
  WS_MQTT_CONNECTED = 0,           // Connected
  WS_MQTT_INVALID_PROTOCOL = 1,    // Invalid mqtt protocol
  WS_MQTT_INVALID_CID = 2,         // Client id rejected
  WS_MQTT_SERVICE_UNAVALIABLE = 3, // Malformed user/pass
  WS_MQTT_INVALID_USER_PASS = 4,   // Unauthorized access to resource
  WS_MQTT_UNAUTHORIZED = 5,        // MQTT service unavailable
  WS_MQTT_THROTTLED = 6,           // Account throttled
  WS_MQTT_BANNED = 7               // Account banned
} ws_mqtt_status_t;

/** Defines the Wippersnapper client's hardware registration status */
typedef enum {
  WS_BOARD_DEF_IDLE,
  WS_BOARD_DEF_SEND_FAILED,
  WS_BOARD_DEF_SENT,
  WS_BOARD_DEF_OK,
  WS_BOARD_DEF_INVALID,
  WS_BOARD_DEF_UNSPECIFIED
} ws_board_status_t;

/* MQTT Configuration */
#define WS_KEEPALIVE_INTERVAL 4 ///< Session keepalive interval time, in seconds
#define WS_KEEPALIVE_INTERVAL_MS                                               \
  4000 ///< Session keepalive interval time, in milliseconds

#define WS_MQTT_MAX_PAYLOAD_SIZE                                               \
  256 ///< MAXIMUM expected payload size, in bytes

class Wippersnapper_Registration;
class Wippersnapper_DigitalGPIO;
class Wippersnapper_AnalogIO;
class Wippersnapper_FS;
class Wippersnapper_ESP32_nvs;
class WipperSnapper_Component_I2C;

/**************************************************************************/
/*!
    @brief  Class that provides storage and functions for the Adafruit IO
            Wippersnapper interface.
*/
/**************************************************************************/
class Wippersnapper {
public:
  Wippersnapper();
  virtual ~Wippersnapper();

  void provision();

  // Status LED
  bool statusLEDInit();
  void statusLEDDeinit();
  void setStatusLEDColor(uint32_t color);
  void statusLEDBlink(ws_led_status_t statusState);
  bool usingStatusNeoPixel =
      false; // True if status LED is using the status neopixel
  bool usingStatusDotStar =
      false;                   // True if status LED is using the status dotstar
  bool usingStatusLED = false; // True if status LED is using the built-in LED

  void set_user_key(const char *aio_username, const char *aio_key);
  void set_user_key();

  virtual void set_ssid_pass(const char *ssid, const char *ssidPassword);
  virtual void set_ssid_pass();

  void connect();
  virtual void _connect();

  void disconnect();
  virtual void _disconnect();

  virtual void setUID();
  virtual void setupMQTTClient(const char *clientID);

  virtual ws_status_t networkStatus();
  ws_status_t status();
  ws_status_t mqttStatus();
  ws_board_status_t getBoardStatus();

  bool buildWSTopics();
  void subscribeWSTopics();
  bool buildErrorTopics();
  void subscribeErrorTopics();

  // Performs board registration FSM
  bool registerBoard(uint8_t retries);

  // run() loop //
  ws_status_t run();
  ws_status_t checkNetworkConnection();
  ws_status_t checkMQTTConnection(uint32_t timeStart);
  void ping();

  // MQTT topic callbacks //
  // Decodes a signal message
  bool decodeSignalMsg(
      wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg);

  // Encodes a pin event message
  bool
  encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
                 wippersnapper_pin_v1_Mode pinMode, uint8_t pinName,
                 int pinVal);

  // Pin configure message
  bool configurePinRequest(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);

  // I2C
  void addNewI2CComponent(int32_t sdaPin, int32_t sclPin, int32_t portNum = 0,
                          uint32_t frequency = 100000U);

  // Decoder for i2c signal incoming
  void decodeMsgSignalI2C();

  uint8_t _buffer[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< Shared buffer to save callback
                                                payload */
  uint8_t
      _buffer_outgoing[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< buffer which contains
                                                     outgoing payload data */
  uint16_t bufSize; /*!< Length of data inside buffer */

  ws_board_status_t _boardStatus; ///< Hardware's registration status

  Wippersnapper_Registration *_registerBoard =
      NULL;                                ///< Instance of registration class
  Wippersnapper_DigitalGPIO *_digitalGPIO; ///< Instance of digital gpio class
  Wippersnapper_AnalogIO *_analogIO;       ///< Instance of analog io class
  Wippersnapper_FS *_fileSystem;           ///< Instance of filesystem class
  Wippersnapper_ESP32_nvs *_nvs;           ///< Instance of nvs

  // I2C WIP - dirty!
  std::vector<WipperSnapper_Component_I2C *> i2cComponents;
  WipperSnapper_Component_I2C *_i2cPort0 = NULL;
  WipperSnapper_Component_I2C *_i2cPort1 = NULL;

  uint8_t _uid[6];          /*!< Unique network iface identifier */
  char sUID[9];             /*!< Unique network iface identifier */
  const char *_boardId;     /*!< Adafruit IO+ board string */
  Adafruit_MQTT *_mqtt;     /*!< Reference to Adafruit_MQTT, _mqtt. */
  char *_topic_description; /*!< MQTT topic for the device description  */

  const char *_mqtt_broker = IO_MQTT_SERVER; /*!< MQTT Broker URL */
  uint16_t _mqtt_port = 8883;                /*!< MQTT Broker URL */

  // AIO credentials
  const char *_username; /*!< Adafruit IO username */
  const char *_key;      /*!< Adafruit IO key */

  // WiFi credentials
  const char *_network_ssid; /*!< WiFi network SSID */
  const char *_network_pass; /*!< WiFi network password*/

  int32_t totalDigitalPins; /*!< Total number of digital-input capable pins */

  char *_topic_signal_device; /*!< Device->Wprsnpr messages */

  wippersnapper_signal_v1_CreateSignalRequest
      _incomingSignalMsg; /*!< Incoming signal message from broker */

  // i2c signal msg
  wippersnapper_signal_v1_I2CRequest msgSignalI2C;

  char *throttleMessage; /*!< Pointer to throttle message data. */
  int throttleTime;      /*!< Total amount of time to throttle the device, in
                            milliseconds. */

private:
  void _init();

protected:
  ws_status_t _status = WS_IDLE;   /*!< Adafruit IO connection status */
  uint32_t _last_mqtt_connect = 0; /*!< Previous time when client connected to
                                          Adafruit IO, in milliseconds. */
  uint32_t _prv_ping = 0;    /*!< Previous time when client pinged Adafruit IO's
                                MQTT broker, in milliseconds. */
  uint32_t _prvKATBlink = 0; /*!< Previous time when client pinged Adafruit IO's
                             MQTT broker, in milliseconds. */

  // Device information
  const char *_deviceId; /*!< Adafruit IO+ device identifier string */
  char *_device_uid;     /*!< Unique device identifier  */

  // MQTT topics
  char *_topic_description_status; /*!< MQTT subtopic carrying the description
                                      status resp. from the broker */
  char *_topic_signal_brkr;        /*!< Wprsnpr->Device messages */
  char *_topic_signal_i2c_brkr;   /*!< Topic carries messages from a device to a
                                     broker. */
  char *_topic_signal_i2c_device; /*!< Topic carries messages from a broker to a
                                     device. */

  Adafruit_MQTT_Subscribe
      *_topic_description_sub; /*!< Subscription for registration topic. */
  Adafruit_MQTT_Publish
      *_topic_signal_device_pub; /*!< Subscription for D2C signal topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_brkr_sub; /*!< Subscription for C2D signal topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_i2c_sub; /*!< Subscribes to signal's I2C topic. */

  char *_err_topic;      /*!< Adafruit IO MQTT error message topic. */
  char *_throttle_topic; /*!< Adafruit IO MQTT throttle message topic. */
  Adafruit_MQTT_Subscribe
      *_err_sub; /*!< Subscription to Adafruit IO Error topic. */
  Adafruit_MQTT_Subscribe
      *_throttle_sub; /*!< Subscription to Adafruit IO Throttle topic. */

  wippersnapper_signal_v1_CreateSignalRequest
      _outgoingSignalMsg; /*!< Outgoing signal message from device */
};

extern Wippersnapper WS; ///< Global member variable for callbacks

#endif // ADAFRUIT_WIPPERSNAPPER_H