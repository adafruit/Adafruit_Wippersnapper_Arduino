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

// Message structures
#include <wippersnapper/description/v1/description.pb.h>    // description.proto
#include <wippersnapper/signal/v1/signal.pb.h>              // signal.proto

// External libraries
#include "Adafruit_MQTT.h"
#include "Arduino.h"
#include <arduino-timer.h>

// Internal libraries
#include "Wippersnapper_Boards.h"

// Reserved Wippersnapper topics
#define TOPIC_WS "/wprsnpr/"            ///< Global /wprsnpr/ topic
#define TOPIC_DESCRIPTION "/info/"      ///< Device description topic
#define TOPIC_SIGNALS     "/signals/"   ///< Device signals topic

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

// Adafruit IO Production SSL Fingerprint
//#define WS_SSL_FINGERPRINT \
//  "59 3C 48 0A B1 8B 39 4E 0D 58 50 47 9A 13 55 60 CC A0 1D AF"

// Adafruit IO Staging SSL Fingerprint
// Fingerprint for io.adafruit.us staging server
#define WS_SSL_FINGERPRINT \
    "CE DC 02 4C B1 1C AE 26 62 EE 55 64 9E 14 F5 A8 3C 45 AE 6E"

class Wippersnapper {

    public:
        Wippersnapper(const char *aio_username, const char *aio_key);
        virtual ~Wippersnapper();

        void connect();
        virtual void _connect() = 0;

        void disconnect();
        virtual void _disconnect() = 0;

        virtual void setUID() = 0;
        virtual void setupMQTTClient(const char *clientID) = 0;

        const __FlashStringHelper *statusText();
        virtual ws_status_t networkStatus() = 0;
        ws_status_t status();
        ws_status_t mqttStatus();
        ws_board_status_t getBoardStatus();

        void generate_feeds(); // Generate device-specific WS feeds

        bool sendBoardDescription();
        bool sendGetHardwareDescription(uint8_t retries);

        ws_status_t checkNetworkConnection(uint32_t timeStart);
        ws_status_t checkMQTTConnection(uint32_t timeStart);
        void ping();
        ws_status_t run();

        // MQTT topic callbacks
        static void cbSignalTopic(char *data, uint16_t len);

        // Signal message
        static bool cbSignalMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);
        bool decodeSignalMsg(wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg);

        // Pin configure message
        static bool cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);
        static bool configPinReq(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg);

        // Pin event message
        static bool cbDecodePinEventMsg(pb_istream_t *stream, const pb_field_t *field, void **arg);
        static bool writePinEvent(wippersnapper_pin_v1_PinEvent *pinEventMsg);

        // Adafruit IO Credentials
        const char *_username; /*!< Adafruit IO Username. */
        const char *_key;      /*!< Adafruit IO Key. */

        static uint16_t bufSize;
        static uint8_t _buffer[128]; /*!< Shared buffer to save callback payload */
        uint8_t _buffer_state[128]; /*!< Holds previous contents of static _buffer */

        // Protobuf helpers
        bool encode_unionmessage(pb_ostream_t *stream, const pb_msgdesc_t *messagetype, void *message);

        // TODO: Possibly remove?
        struct pinInfo {
            char *pinName;
            char *PinNameFull;
            int pinValue;
            int prvPinValue; // holds prv. pin state
        };
        static pinInfo ws_pinInfo;

        static Timer<16, millis, char *> t_timer;

        static char timerPin[3];

    private:
        void _init();

    protected:
        ws_status_t _status = WS_IDLE; /*!< Adafruit IO connection status */
        uint32_t _last_mqtt_connect = 0; /*!< Previous time when client connected to
                                                Adafruit IO, in milliseconds */
        uint32_t _prv_ping = 0;

        Adafruit_MQTT *_mqtt;                         /*!< Reference to Adafruit_MQTT, _mqtt. */

        // PoC Server
        // const char *_mqtt_broker = "2.tcp.ngrok.io"; /*!< MQTT Broker URL */
        // uint16_t _mqtt_port = 18653;                 /*!< MQTT Broker URL */

        // Staging Server
        const char *_mqtt_broker = "io.adafruit.us"; /*!< MQTT Broker URL */
        uint16_t _mqtt_port = 8883;                  /*!< MQTT Broker URL */

        // Production Server
        // const char *_mqtt_broker = "io.adafruit.com"; /*!< MQTT Broker URL */
        // uint16_t _mqtt_port = 8883;                   /*!< MQTT Broker URL */

        // Device information
        const char *_deviceId; /*!< Adafruit IO+ device identifier string */
        const char *_boardId;  /*!< Adafruit IO+ board string */
        uint16_t _hw_vid;      /*!< USB vendor identifer */
        uint16_t _hw_pid;      /*!< USB product identifier */
        uint8_t _uid[6];       /*!< Unique network iface identifier */
        char *_device_uid;     /*!< Unique device identifier  */
        char sUID[9];

        // MQTT topics
        char *_topic_description;        /*!< MQTT topic for the device description  */
        char *_topic_description_status; /*!< MQTT subtopic carrying the description status resp. from the broker */
        char *_topic_signal_brkr;        /*!< Wprsnpr->Device messages */
        char *_topic_signal_device;      /*!< Device->Wprsnpr messages */

        Adafruit_MQTT_Subscribe *_topic_description_sub;
        Adafruit_MQTT_Publish *_topic_signal_device_pub;
        Adafruit_MQTT_Subscribe *_topic_signal_brkr_sub;
        Adafruit_MQTT_Subscribe *_subscription;

        static char _value[45]; /*!< Data to send back to Wippersnapper, max. IO data len */
        static char _prv_value[45]; /*!< Data to send back to Wippersnapper, max. IO data len */
};

#endif // ADAFRUIT_WIPPERSNAPPER_H