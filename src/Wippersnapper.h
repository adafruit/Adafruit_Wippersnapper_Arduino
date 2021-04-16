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

// Internal
#include "Wippersnapper_Boards.h"
#include "Wippersnapper_Registration.h"

// External libraries
#include "Adafruit_MQTT.h" // MQTT Client
#include "Arduino.h" // Wiring
#include <Adafruit_NeoPixel.h>

// Reserved Wippersnapper topics
#define TOPIC_WS            "/wprsnpr/"   ///< Global /wprsnpr/ topic
#define TOPIC_DESCRIPTION   "/info/"      ///< Device description topic
#define TOPIC_SIGNALS       "/signals/"   ///< Device signals topic

#define WS_DEBUG
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

// Adafruit IO Status States
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
  WS_BOARD_DESC_INVALID = 25       // Unable to send board description
} ws_status_t;

// Wippersnapper board definition status
typedef enum {
    WS_BOARD_DEF_IDLE,
    WS_BOARD_DEF_SEND_FAILED,
    WS_BOARD_DEF_SENT,
    WS_BOARD_DEF_OK,
    WS_BOARD_DEF_INVALID,
    WS_BOARD_DEF_UNSPECIFIED
} ws_board_status_t;

// Holds data about a digital input timer
// members assigned from a PinConfigureRequest
struct digitalInputPin {
    uint8_t pinName; // Pin name
    long timerInterval; // timer interval, in millis, -1 if disabled.
    long timerIntervalPrv; // time timer was previously serviced, in millis
    int prvPinVal; // Previous pin value
};

#define MAX_DIGITAL_TIMERS 5

// Adafruit IO Production SSL Fingerprint
//#define WS_SSL_FINGERPRINT \
//  "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF"

// Adafruit IO Staging SSL Fingerprint
// Fingerprint for io.adafruit.us staging server
#define WS_SSL_FINGERPRINT \
    "CE DC 02 4C B1 1C AE 26 62 EE 55 64 9E 14 F5 A8 3C 45 AE 6E"

/* MQTT Configuration */
// Keep Alive interval, in ms
#define WS_KEEPALIVE_INTERVAL 10000

// MAXIMUM MQTT expected payload size, in bytes
#define WS_MQTT_MAX_PAYLOAD_SIZE 128


class Wippersnapper_Registration;

class Wippersnapper {


    public:
        Wippersnapper();
        virtual ~Wippersnapper();

        void set_user_key(const char *aio_username, const char *aio_key);

        void set_ssid_pass(char *ssid, const char *ssidPassword);

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

        // Generates Wippersnapper MQTT feeds
        void generate_subscribe_feeds();

        // Performs board registration FSM
        bool registerBoard(uint8_t retries);

        // run() loop //
        ws_status_t run();
        ws_status_t checkNetworkConnection(uint32_t timeStart);
        ws_status_t checkMQTTConnection(uint32_t timeStart);
        // Pumps message loop
        bool processSignalMessages(int16_t timeout);

        // MQTT topic callbacks //

        // Signal message // 
        // Called when a new signal message has been received
        //static bool cbSignalMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);
        
        // Decodes a signal message
        bool decodeSignalMsg(wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg);

        // Encodes a signal message
        bool encodeSignalMsg(uint8_t signalPayloadType);

        // Encodes a pin event message
        bool encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg, wippersnapper_pin_v1_Mode pinMode, uint8_t pinName, int pinVal);

        // Pin configure message
        bool cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);
        bool configurePinRequest(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);

        // Decodes list of pin events
        //static bool cbDecodePinEventMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);

        // Digital Input
        static void attachDigitalPinTimer(uint8_t pinName, float interval);
        static void detachDigitalPinTimer(uint8_t pinName);


        uint8_t _buffer[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< Shared buffer to save callback payload */
        uint8_t _buffer_outgoing[WS_MQTT_MAX_PAYLOAD_SIZE]; /*!< buffer which contains outgoing payload data */
        uint16_t bufSize; /*!< Length of data inside buffer */

        static ws_board_status_t _boardStatus;

        Wippersnapper_Registration *_registerBoard;

        Adafruit_NeoPixel pixels; /*!< NeoPixel strand */

        uint8_t _uid[6];       /*!< Unique network iface identifier */

        char sUID[9];
        const char *_boardId;  /*!< Adafruit IO+ board string */
        Adafruit_MQTT *_mqtt;                         /*!< Reference to Adafruit_MQTT, _mqtt. */
        char *_topic_description;        /*!< MQTT topic for the device description  */

        // Staging Server
        const char *_mqtt_broker = "io.adafruit.us"; /*!< MQTT Broker URL */
        uint16_t _mqtt_port = 8883;                  /*!< MQTT Broker URL */

        // AIO Credentials
        const char *_username;
        const char *_key;

        int32_t total_gpio_pins; /*!< Total number of hardware's GPIO pins */
        int32_t total_analog_pins; /*!< Total number of hardware's analog input pins */
        digitalInputPin* _digital_input_pins; /*!< Array of gpio pin objects */

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
        char *_topic_signal_device;      /*!< Device->Wprsnpr messages */

        Adafruit_MQTT_Subscribe *_topic_description_sub;
        Adafruit_MQTT_Publish *_topic_signal_device_pub;
        Adafruit_MQTT_Subscribe *_topic_signal_brkr_sub;

        static char _value[45]; /*!< Data to send back to Wippersnapper, max. IO data len */
        static char _prv_value[45]; /*!< Data to send back to Wippersnapper, max. IO data len */

        wippersnapper_signal_v1_CreateSignalRequest _incomingSignalMsg; /*!< Incoming signal message from broker */
        wippersnapper_signal_v1_CreateSignalRequest _outgoingSignalMsg; /*!< Outgoing signal message from device */

        // Holds info about all digital input timers
        // TODO: This is currently fixed at "2" pins, we need to
        // transmit the # of pins from the broker during registration
        // and dynamically create this
       static digitalInputPin _timersDigital[MAX_DIGITAL_TIMERS];
};

extern Wippersnapper WS;

#endif // ADAFRUIT_WIPPERSNAPPER_H