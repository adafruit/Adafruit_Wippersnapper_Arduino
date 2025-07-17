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
 * Copyright (c) Brent Rubell 2020-2025 for Adafruit Industries.
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

// External libraries
#include "Adafruit_MQTT.h"      // MQTT Client
#include "Adafruit_SleepyDog.h" // Watchdog
#include "Arduino.h"            // Wiring
#include <SPI.h>                // SPI

// Wippersnapper API Helpers
#include "Wippersnapper_Boards.h"
#include "components/statusLED/Wippersnapper_StatusLED.h"
#include "provisioning/ConfigJson.h"

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

/**************************************************************************/
/*!
    @brief  Retry a function until a condition is met or a timeout is reached.
    @param  func
            The function to retry.
    @param  result_type
            The type of the result of the function.
    @param  result_var
            The variable to store the last result of the function.
    @param  condition
            The condition to check the result against.
    @param  timeout
            The maximum time to retry the function.
    @param  interval
            The time to wait between retries.
    @param  ...
            The arguments to pass to the function.
*/
/**************************************************************************/
#define RETRY_FUNCTION_UNTIL_TIMEOUT(func, result_type, result_var, condition, \
                                     timeout, interval, ...)                   \
  {                                                                            \
    unsigned long startTime = millis();                                        \
    while (millis() - startTime < timeout) {                                   \
      result_type result_var = func(__VA_ARGS__);                              \
      if (condition(result_var)) {                                             \
        break;                                                                 \
      }                                                                        \
      if (startTime > millis()) {                                              \
        startTime = millis(); /* if rollover */                                \
      }                                                                        \
      WS_DELAY_WITH_WDT(interval);                                             \
    }                                                                          \
  } ///< Retry a function until a condition is met or a timeout is reached.

// Wippersnapper pb helpers
#include <nanopb/ws_pb_helpers.h>

// Wippersnapper components
#include "components/analogIO/Wippersnapper_AnalogIO.h"
#include "components/digitalIO/Wippersnapper_DigitalGPIO.h"
#include "components/i2c/WipperSnapper_I2C.h"

// Includes for ESP32-only
#ifdef ARDUINO_ARCH_ESP32
#include "components/ledc/ws_ledc.h"
#include <Esp.h>
#endif

// Display
#ifdef USE_DISPLAY
#include "display/ws_display_driver.h"
#include "display/ws_display_ui_helper.h"
#endif

#include "components/ds18x20/ws_ds18x20.h"
#include "components/pixels/ws_pixels.h"
#include "components/pwm/ws_pwm.h"
#include "components/servo/ws_servo.h"
#include "components/uart/ws_uart.h"

#if defined(USE_TINYUSB)
#include "provisioning/tinyusb/Wippersnapper_FS.h"
#endif

#if defined(USE_LITTLEFS)
#include "provisioning/littlefs/WipperSnapper_LittleFS.h"
#endif

#define WS_VERSION                                                             \
  "1.0.0-beta.106" ///< WipperSnapper app. version (semver-formatted)

// Reserved Adafruit IO MQTT topics
#define TOPIC_IO_THROTTLE "/throttle" ///< Adafruit IO Throttle MQTT Topic
#define TOPIC_IO_ERRORS "/errors"     ///< Adafruit IO Error MQTT Topic

// Reserved Wippersnapper topics
#define TOPIC_WS "/wprsnpr/"      ///< WipperSnapper topic
#define TOPIC_INFO "/info/"       ///< Registration sub-topic
#define TOPIC_SIGNALS "/signals/" ///< Signals sub-topic
#define TOPIC_I2C "/i2c"          ///< I2C sub-topic
#define MQTT_TOPIC_PIXELS_DEVICE                                               \
  "/signals/device/pixel" ///< Pixels device->broker topic
#define MQTT_TOPIC_PIXELS_BROKER                                               \
  "/signals/broker/pixel" ///< Pixels broker->device topic

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

#ifdef ARDUINO_ARCH_RP2040
#define WS_WDT_TIMEOUT 8388 ///< RP2040 Max WDT timeout
#else
#define WS_WDT_TIMEOUT 60000 ///< WDT timeout
#endif

#define WS_MAX_ALT_WIFI_NETWORKS 3 ///< Maximum number of alternative networks
/* MQTT Configuration */
#define WS_KEEPALIVE_INTERVAL_MS                                               \
  5000 ///< Session keepalive interval time, in milliseconds

#define WS_MQTT_MAX_PAYLOAD_SIZE                                               \
  512 ///< MAXIMUM expected payload size, in bytes

class Wippersnapper_DigitalGPIO;
class Wippersnapper_AnalogIO;
class Wippersnapper_FS;
class WipperSnapper_LittleFS;
#ifdef USE_DISPLAY
class ws_display_driver;
class ws_display_ui_helper;
#endif
#ifdef ARDUINO_ARCH_ESP32
class ws_ledc;
#endif
class WipperSnapper_Component_I2C;
class ws_servo;
class ws_pwm;
class ws_ds18x20;
class ws_pixels;
class ws_uart;

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

  bool lockStatusNeoPixel; ///< True if status LED is using the status neopixel
  bool lockStatusDotStar;  ///< True if status LED is using the status dotstar
  bool lockStatusLED;      ///< True if status LED is using the built-in LED
  float status_pixel_brightness =
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
  ws_board_status_t getBoardStatus();

  bool generateDeviceUID();
  bool generateWSTopics();
  bool generateWSErrorTopics();

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
                 ws_led_status_t ledStatusColor = WS_LED_STATUS_ERROR_RUNTIME,
                 int seconds_until_reboot = 25);
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
  bool configureDigitalPinReq(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);
  bool configAnalogInPinReq(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);

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

  ws_board_status_t _boardStatus =
      WS_BOARD_DEF_IDLE; ///< Hardware's registration status

  // TODO: We really should look at making these static definitions, not dynamic
  // to free up space on the heap
  Wippersnapper_DigitalGPIO *_digitalGPIO; ///< Instance of digital gpio class
  Wippersnapper_AnalogIO *_analogIO;       ///< Instance of analog io class
  Wippersnapper_FS *_fileSystem; ///< Instance of Filesystem (native USB)
  WipperSnapper_LittleFS
      *_littleFS; ///< Instance of LittleFS Filesystem (non-native USB)
#ifdef USE_DISPLAY
  ws_display_driver *_display = nullptr; ///< Instance of display driver class
  ws_display_ui_helper *_ui_helper =
      nullptr; ///< Instance of display UI helper class
#endif
  ws_pixels *_ws_pixelsComponent; ///< ptr to instance of ws_pixels class
  ws_pwm *_pwmComponent;          ///< Instance of pwm class
  ws_servo *_servoComponent;      ///< Instance of servo class
  ws_ds18x20 *_ds18x20Component;  ///< Instance of DS18x20 class
  ws_uart *_uartComponent;        ///< Instance of UART class

  // TODO: does this really need to be global?
  uint8_t _macAddr[6];  /*!< Unique network iface identifier */
  char sUID[13];        /*!< Unique network iface identifier */
  const char *_boardId; /*!< Adafruit IO+ board string */
  Adafruit_MQTT *_mqtt; /*!< Reference to Adafruit_MQTT, _mqtt. */

  secretsConfig _config; /*!< Wippersnapper secrets.json as a struct. */
  networkConfig _multiNetworks[3]; /*!< Wippersnapper networks as structs. */
  bool _isWiFiMulti = false; /*!< True if multiple networks are defined. */

  // TODO: Does this need to be within this class?
  int32_t totalDigitalPins; /*!< Total number of digital-input capable pins */

  char *_topic_description = NULL; /*!< MQTT topic for the device description */
  char *_topic_signal_device = NULL;   /*!< Device->Wprsnpr messages */
  char *_topic_signal_i2c_brkr = NULL; /*!< Topic carries messages from a device
                                   to a broker. */
  char *_topic_signal_i2c_device = NULL;   /*!< Topic carries messages from a
                                       broker to a device. */
  char *_topic_signal_servo_brkr = NULL;   /*!< Topic carries messages from a
                                     device   to a broker. */
  char *_topic_signal_servo_device = NULL; /*!< Topic carries messages from a
                                     broker to a device. */
  char *_topic_signal_pwm_brkr =
      NULL; /*!< Topic carries PWM messages from a device to a broker. */
  char *_topic_signal_pwm_device =
      NULL; /*!< Topic carries PWM messages from a broker to a device. */
  char *_topic_signal_ds18_brkr = NULL; /*!< Topic carries ds18x20 messages from
                                   a device to a broker. */
  char *_topic_signal_ds18_device = NULL;   /*!< Topic carries ds18x20 messages
                                       from a broker to a device. */
  char *_topic_signal_pixels_brkr = NULL;   /*!< Topic carries pixel messages */
  char *_topic_signal_pixels_device = NULL; /*!< Topic carries pixel messages */
  char *_topic_signal_uart_brkr = NULL;     /*!< Topic carries UART messages */
  char *_topic_signal_uart_device = NULL;   /*!< Topic carries UART messages */

  wippersnapper_signal_v1_CreateSignalRequest
      _incomingSignalMsg; /*!< Incoming signal message from broker */
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
  wippersnapper_signal_v1_PWMRequest msgPWM =
      wippersnapper_signal_v1_PWMRequest_init_zero; ///< PWM request wrapper
                                                    ///< message.

  // pixels signal message
  wippersnapper_signal_v1_PixelsRequest
      msgPixels; ///< PixelsRequest wrapper message

  wippersnapper_signal_v1_UARTRequest
      msgSignalUART; ///< UARTReq wrapper message

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

  Adafruit_MQTT_Subscribe *_topic_description_sub; /*!< Subscription callback
                                                      for registration topic. */
  Adafruit_MQTT_Publish *_topic_signal_device_pub; /*!< Subscription callback
                                                      for D2C signal topic. */
  Adafruit_MQTT_Subscribe *_topic_signal_brkr_sub; /*!< Subscription callback
                                                      for C2D signal topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_i2c_sub; /*!< Subscription callback for I2C topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_servo_sub; /*!< Subscription callback for servo topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_pwm_sub; /*!< Subscription callback for pwm topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_ds18_sub; /*!< Subscribes to signal's ds18x20 topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_pixels_sub; /*!< Subscribes to pixel device topic. */
  Adafruit_MQTT_Subscribe
      *_topic_signal_uart_sub; /*!< Subscribes to signal's UART topic. */

  Adafruit_MQTT_Subscribe
      *_err_sub; /*!< Subscription to Adafruit IO Error topic. */
  Adafruit_MQTT_Subscribe
      *_throttle_sub; /*!< Subscription to Adafruit IO Throttle topic. */

  wippersnapper_signal_v1_CreateSignalRequest
      _outgoingSignalMsg; /*!< Outgoing signal message from device */
};
extern Wippersnapper WS; ///< Global member variable for callbacks

#endif // ADAFRUIT_WIPPERSNAPPER_H
