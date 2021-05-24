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
 * Written by Brent Rubell for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#ifndef WIPPERSNAPPER_H
#define WIPPERSNAPPER_H

// Nanopb dependencies
#include <pb.h>
#include <nanopb/pb_common.h>
#include <nanopb/pb_encode.h>
#include <nanopb/pb_decode.h>

#include <wippersnapper/description/v1/description.pb.h>    // description.proto
#include <wippersnapper/signal/v1/signal.pb.h>              // signal.proto
#include <wippersnapper/pin/v1/pin.pb.h>                    // pin.proto

// Wippersnapper API Helpers
#include "Wippersnapper_Boards.h"
#include "Wippersnapper_Registration.h"

// Wippersnapper GPIO
#include "Wippersnapper_DigitalGPIO.h"
#include "Wippersnapper_AnalogIO.h"

// External libraries
#include "Adafruit_MQTT.h" // MQTT Client
#include "Arduino.h" // Wiring
#include <Adafruit_NeoPixel.h>

// Reserved Adafruit IO MQTT topics
#define TOPIC_IO_THROTTLE "/throttle"  ///< Adafruit IO Throttle MQTT Topic
#define TOPIC_IO_ERRORS "/errors"      ///< Adafruit IO Error MQTT Topic

// Reserved Wippersnapper topics
#define TOPIC_WS            "/wprsnpr/"   ///< Global /wprsnpr/ topic
#define TOPIC_DESCRIPTION   "/info/"      ///< Device description topic
#define TOPIC_SIGNALS       "/signals/"   ///< Device signals topic

#define WS_DEBUG          ///< Define to enable debugging to serial terminal
#define WS_PRINTER Serial ///< Where debug messages will be printed

// Define actual debug output functions when necessary.
#ifdef WS_DEBUG
#define WS_DEBUG_PRINT(...)                                                   \
  { WS_PRINTER.print(__VA_ARGS__); } ///< Prints debug output.
#define WS_DEBUG_PRINTLN(...)                                                 \
  { WS_PRINTER.println(__VA_ARGS__); } ///< Prints line from debug output.
#else
#define WS_DEBUG_PRINT(...)                                                   \
  {} ///< Prints debug output
#define WS_DEBUG_PRINTLN(...)                                                 \
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
  WS_BOARD_DESC_INVALID = 25,       // Unable to send board description
  WS_BOARD_RESYNC_FAILED = 26       // Board sync failure
} ws_status_t;

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
#define WS_KEEPALIVE_INTERVAL    2 ///< Session keepalive interval time, in seconds
#define WS_KEEPALIVE_INTERVAL_MS 2000 ///< Session keepalive interval time, in milliseconds

#define WS_MQTT_MAX_PAYLOAD_SIZE 128 ///< MAXIMUM expected payload size, in bytes

class Wippersnapper_Registration;
class Wippersnapper_DigitalGPIO;
class Wippersnapper_AnalogIO;

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

        void set_user_key(const char *aio_username, const char *aio_key);
        virtual void set_ssid_pass(char *ssid, const char *ssidPassword);

        void connect();
        virtual void _connect();

        void disconnect();
        virtual void _disconnect();

        virtual void setUID();
        virtual void setupMQTTClient(const char *clientID);

        const __FlashStringHelper *statusText();
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
        bool decodeSignalMsg(wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg);

        // Encodes a pin event message
        bool encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg, wippersnapper_pin_v1_Mode pinMode, uint8_t pinName, int pinVal);

        // Pin configure message
        bool cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);
        bool configurePinRequest(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);

        uint8_t _buffer[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< Shared buffer to save callback payload */
        uint8_t _buffer_outgoing[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< buffer which contains outgoing payload data */
        uint16_t bufSize; /*!< Length of data inside buffer */

        ws_board_status_t _boardStatus;

        Wippersnapper_Registration *_registerBoard;  /*!< Instance of registration class */
        Wippersnapper_DigitalGPIO *_digitalGPIO;     /*!< Instance of digital gpio class */
        Wippersnapper_AnalogIO *_analogIO;           /*!< Instance of analog io class */

        // TODO: move neopixel into its own class
        Adafruit_NeoPixel pixels; /*!< NeoPixel */

        uint8_t _uid[6]; /*!< Unique network iface identifier */
        char sUID[9]; /*!< Unique network iface identifier */
        const char *_boardId;       /*!< Adafruit IO+ board string */
        Adafruit_MQTT *_mqtt;       /*!< Reference to Adafruit_MQTT, _mqtt. */
        char *_topic_description;   /*!< MQTT topic for the device description  */

        // Staging Server
        const char *_mqtt_broker = "io.adafruit.us"; /*!< MQTT Broker URL */
        uint16_t _mqtt_port = 8883;                  /*!< MQTT Broker URL */

        // AIO Credentials
        const char *_username;  /*!< Adafruit IO username */
        const char *_key;       /*!< Adafruit IO key */

        int32_t totalDigitalPins;     /*!< Total number of digital-input capable pins */

        char *_topic_signal_device;      /*!< Device->Wprsnpr messages */

        wippersnapper_signal_v1_CreateSignalRequest _incomingSignalMsg; /*!< Incoming signal message from broker */

    private:
        void _init();

    protected:
        ws_status_t _status = WS_IDLE; /*!< Adafruit IO connection status */
        uint32_t _last_mqtt_connect = 0; /*!< Previous time when client connected to
                                                Adafruit IO, in milliseconds */
        uint32_t _prv_ping = 0;

        // PoC Server
        // const char *_mqtt_broker = "2.tcp.ngrok.io"; /*!< MQTT Broker URL */
        // uint16_t _mqtt_port = 18653;                 /*!< MQTT Broker URL */

        // Production Server
        // const char *_mqtt_broker = "io.adafruit.com"; /*!< MQTT Broker URL */
        // uint16_t _mqtt_port = 8883;                   /*!< MQTT Broker URL */

        // Device information
        const char *_deviceId; /*!< Adafruit IO+ device identifier string */
        char *_device_uid;     /*!< Unique device identifier  */


        // MQTT topics
        char *_topic_description_status; /*!< MQTT subtopic carrying the description status resp. from the broker */
        char *_topic_signal_brkr;        /*!< Wprsnpr->Device messages */

        Adafruit_MQTT_Subscribe *_topic_description_sub;
        Adafruit_MQTT_Publish *_topic_signal_device_pub;
        Adafruit_MQTT_Subscribe *_topic_signal_brkr_sub;

        char *_err_topic;                       /*!< Adafruit IO MQTT error message topic. */
        char *_throttle_topic;                  /*!< Adafruit IO MQTT throttle message topic. */
        Adafruit_MQTT_Subscribe *_err_sub;      /*!< Subscription to Adafruit IO Error topic. */
        Adafruit_MQTT_Subscribe *_throttle_sub; /*!< Subscription to Adafruit IO Throttle topic. */

        
        wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg; /*!< Outgoing signal message from device */

};

extern Wippersnapper WS; ///< Global member variable for callbacks

#endif // ADAFRUIT_WIPPERSNAPPER_H