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
 * Copyright (c) Brent Rubell 2020-2022 for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_H
#define WIPPERSNAPPER_H

// Cpp STD
#include <vector>

// Nanopb dependencies
#include <nanopb/pb_common.h>
#include <nanopb/pb_decode.h>
#include <nanopb/pb_encode.h>
#include <pb.h>

#include <wippersnapper/description/v1/description.pb.h> // description.proto
#include <wippersnapper/signal/v1/signal.pb.h>           // signal.proto

// Wippersnapper API Helpers
#include "Wippersnapper_Boards.h"
#include "components/statusLED/Wippersnapper_StatusLED.h"

// Wippersnapper components
#include "components/analogIO/Wippersnapper_AnalogIO.h"
#include "components/digitalIO/Wippersnapper_DigitalGPIO.h"
#include "components/i2c/WipperSnapper_I2C.h"

// LEDC-Manager, ESP32-only
#ifdef ARDUINO_ARCH_ESP32
#include "components/ledc/ws_ledc.h"
#endif

#include "components/ds18x20/ws_ds18x20.h"
#include "components/servo/ws_servo.h"

// External libraries
#include "Adafruit_MQTT.h" // MQTT Client
#include "Arduino.h"       // Wiring

// Note: These might be better off in their respective wrappers
#include <SPI.h>

#ifndef ESP8266
#include "Adafruit_SleepyDog.h"
#endif

#if defined(USE_TINYUSB)
#include "provisioning/tinyusb/Wippersnapper_FS.h"
#endif

#if defined(USE_LITTLEFS)
#include "provisioning/littlefs/WipperSnapper_LittleFS.h"
#endif

#define WS_VERSION                                                             \
  "1.0.0-beta.54" ///< WipperSnapper app. version (semver-formatted)

// Reserved Adafruit IO MQTT topics
#define TOPIC_IO_THROTTLE "/throttle" ///< Adafruit IO Throttle MQTT Topic
#define TOPIC_IO_ERRORS "/errors"     ///< Adafruit IO Error MQTT Topic

// Reserved Wippersnapper topics
#define TOPIC_WS "/wprsnpr/"      ///< WipperSnapper topic
#define TOPIC_INFO "/info/"       ///< Registration sub-topic
#define TOPIC_SIGNALS "/signals/" ///< Signals sub-topic
#define TOPIC_I2C "/i2c"          ///< I2C sub-topic

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

/** Defines the Wippersnapper client's network status */
typedef enum {
  FSM_NET_IDLE,
  FSM_NET_CONNECTED,
  FSM_MQTT_CONNECTED,
  FSM_NET_CHECK_MQTT,
  FSM_NET_CHECK_NETWORK,
  FSM_NET_ESTABLISH_NETWORK,
  FSM_NET_ESTABLISH_MQTT,
} fsm_net_t;

#define WS_WDT_TIMEOUT 60000 ///< WDT timeout
/* MQTT Configuration */
// TODO: Redundant, we should reference keepalive_interval_ms and just do math
#define WS_KEEPALIVE_INTERVAL 4 ///< Session keepalive interval time, in seconds
#define WS_KEEPALIVE_INTERVAL_MS                                               \
  4000 ///< Session keepalive interval time, in milliseconds

#define WS_MQTT_MAX_PAYLOAD_SIZE                                               \
  512 ///< MAXIMUM expected payload size, in bytes

class Wippersnapper_DigitalGPIO;
class Wippersnapper_AnalogIO;
class Wippersnapper_FS;
class WipperSnapper_LittleFS;
class WipperSnapper_Component_I2C;
#ifdef ARDUINO_ARCH_ESP32
class ws_ledc;
#endif
class ws_servo;
class ws_ds18x20;

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

  bool lockStatusNeoPixel =
      false; ///< True if status LED is using the status neopixel
  bool lockStatusDotStar =
      false; ///< True if status LED is using the status dotstar
  bool lockStatusLED = false; ///< True if status LED is using the built-in LED

  virtual void set_user_key();
  virtual void set_ssid_pass(const char *ssid, const char *ssidPassword);
  virtual void set_ssid_pass();

  virtual void _connect();
  virtual void _disconnect();
  void connect();
  void disconnect();

  virtual void getMacAddr();
  virtual void setupMQTTClient(const char *clientID);

  virtual ws_status_t networkStatus();
  ws_board_status_t getBoardStatus();

  bool buildWSTopics();
  void subscribeWSTopics();
  bool buildErrorTopics();
  void subscribeErrorTopics();

  // Registration API
  bool registerBoard();
  bool encodePubRegistrationReq();
  void decodeRegistrationResp(char *data, uint16_t len);
  void pollRegistrationResp();
  // Configuration API
  void publishPinConfigComplete();

  // run() loop
  ws_status_t run();
  void processPackets();
  void publish(const char *topic, uint8_t *payload, uint16_t bLen,
               uint8_t qos = 0);

  // Networking helpers
  void pingBroker();
  void runNetFSM();

  // WDT helpers
  void enableWDT(int timeoutMS = 0);
  void feedWDT();

  // Error handling helpers
  void haltError(String error,
                 ws_led_status_t ledStatusColor = WS_LED_STATUS_ERROR_RUNTIME);
  void errorWriteHang(String error);

  // MQTT topic callbacks //
  // Decodes a signal message
  bool decodeSignalMsg(
      wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg);

  // Encodes a pin event message
  bool
  encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
                 uint8_t pinName, int pinVal);

  // Pin configure message
  bool configurePinRequest(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);

  // I2C
  std::vector<WipperSnapper_Component_I2C *>
      i2cComponents; ///< Vector containing all I2C components
  WipperSnapper_Component_I2C *_i2cPort0 =
      NULL; ///< WipperSnapper I2C Component for I2C port #0
  WipperSnapper_Component_I2C *_i2cPort1 =
      NULL; ///< WipperSnapper I2C Component for I2C port #1
  bool _isI2CPort0Init =
      false; ///< True if I2C port 0 has been initialized, False otherwise.
  bool _isI2CPort1Init =
      false; ///< True if I2C port 1 has been initialized, False otherwise.

  uint8_t _buffer[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< Shared buffer to save callback
                                                payload */
  uint8_t
      _buffer_outgoing[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< buffer which contains
                                                     outgoing payload data */
  uint16_t bufSize; /*!< Length of data inside buffer */

  ws_board_status_t _boardStatus; ///< Hardware's registration status

  Wippersnapper_DigitalGPIO *_digitalGPIO; ///< Instance of digital gpio class
  Wippersnapper_AnalogIO *_analogIO;       ///< Instance of analog io class
  Wippersnapper_FS *_fileSystem; ///< Instance of Filesystem (native USB)
  WipperSnapper_LittleFS
      *_littleFS; ///< Instance of LittleFS Filesystem (non-native USB)
  ws_servo *_servoComponent;     ///< Instance of servo class
  ws_ds18x20 *_ds18x20Component; ///< Instance of DS18x20 class

  uint8_t _macAddr[6];  /*!< Unique network iface identifier */
  char sUID[13];        /*!< Unique network iface identifier */
  const char *_boardId; /*!< Adafruit IO+ board string */
  Adafruit_MQTT *_mqtt; /*!< Reference to Adafruit_MQTT, _mqtt. */

  const char *_mqttBrokerURL = nullptr; /*!< MQTT Broker URL */
  uint16_t _mqtt_port = 8883;           /*!< MQTT Broker Port */

  // AIO credentials
  const char *_username = NULL; /*!< Adafruit IO username */
  const char *_key = NULL;      /*!< Adafruit IO key */

  // WiFi credentials
  const char *_network_ssid = NULL; /*!< WiFi network SSID */
  const char *_network_pass = NULL; /*!< WiFi network password*/

  int32_t totalDigitalPins; /*!< Total number of digital-input capable pins */

  char *_topic_description =
      NULL; /*!< MQTT topic for the device description  */
  char *_topic_signal_device = NULL;   /*!< Device->Wprsnpr messages */
  char *_topic_signal_i2c_brkr = NULL; /*!< Topic carries messages from a device
                                   to a broker. */
  char *_topic_signal_i2c_device = NULL;   /*!< Topic carries messages from a
                                       broker to a device. */
  char *_topic_signal_servo_brkr = NULL;   /*!< Topic carries messages from a
                                     device   to a broker. */
  char *_topic_signal_servo_device = NULL; /*!< Topic carries messages from a
                                     broker to a device. */
  char *_topic_signal_ds18_brkr = NULL; /*!< Topic carries ds18x20 messages from
                                   a device to a broker. */
  char *_topic_signal_ds18_device = NULL; /*!< Topic carries ds18x20 messages
                                     from a broker to a device. */

  wippersnapper_signal_v1_CreateSignalRequest
      _incomingSignalMsg; /*!< Incoming signal message from broker */

  // i2c signal msg
  wippersnapper_signal_v1_I2CRequest msgSignalI2C =
      wippersnapper_signal_v1_I2CRequest_init_zero; ///< I2C request wrapper
                                                    ///< message

  // ds signal msg
  wippersnapper_signal_v1_Ds18x20Request msgSignalDS =
      wippersnapper_signal_v1_Ds18x20Request_init_zero; ///< DS request message
                                                        ///< wrapper

  // servo message
  wippersnapper_signal_v1_ServoRequest
      msgServo; ///< ServoRequest wrapper message

  char *throttleMessage; /*!< Pointer to throttle message data. */
  int throttleTime;      /*!< Total amount of time to throttle the device, in
                            milliseconds. */

  bool pinCfgCompleted = false; /*!< Did initial pin sync complete? */

// enable LEDC if esp32
#ifdef ARDUINO_ARCH_ESP32
  ws_ledc *_ledc = nullptr; ///< Pointer to LEDC object
#endif

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
  char *_topic_description_status =
      NULL; /*!< MQTT subtopic carrying the description
        status resp. from the broker */
  char *_topic_description_status_complete = NULL; /*!< MQTT topic carrying the
                                               ACK signal from the device to the
                                               broker after registration */
  char *_topic_device_pin_config_complete =
      NULL;                        /*!< MQTT topic carrying the ACK signal
                               from the device to the broker after
                               hardware configuration */
  char *_topic_signal_brkr = NULL; /*!< Wprsnpr->Device messages */
  char *_err_topic = NULL;         /*!< Adafruit IO MQTT error message topic. */
  char *_throttle_topic = NULL; /*!< Adafruit IO MQTT throttle message topic. */

  Adafruit_MQTT_Subscribe
      *_topic_description_sub; /*!< Subscription for registration topic. */
  Adafruit_MQTT_Publish
      *_topic_signal_device_pub; /*!< Subscription for D2C signal topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_brkr_sub; /*!< Subscription for C2D signal topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_i2c_sub; /*!< Subscribes to signal's I2C topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_ds18_sub; /*!< Subscribes to signal's ds18x20 topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_servo_sub; /*!< Subscribes to device's servo topic. */

  Adafruit_MQTT_Subscribe
      *_err_sub; /*!< Subscription to Adafruit IO Error topic. */
  Adafruit_MQTT_Subscribe
      *_throttle_sub; /*!< Subscription to Adafruit IO Throttle topic. */

  wippersnapper_signal_v1_CreateSignalRequest
      _outgoingSignalMsg; /*!< Outgoing signal message from device */
};
extern Wippersnapper WS; ///< Global member variable for callbacks

#endif // ADAFRUIT_WIPPERSNAPPER_H
