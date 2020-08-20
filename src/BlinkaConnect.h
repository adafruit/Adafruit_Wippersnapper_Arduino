/*!
 * @file BlinkaConnect.h
 *
 * This is the documentation for Adafruit's BlinkaConnect wrapper for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit IO+ BlinkaConnect IoT platform.
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

#ifndef BLINKACONNECT_H
#define BLINKACONNECT_H

// Nanopb
#include <nanopb/pb_common.h>
#include <nanopb/pb.h>
#include <nanopb/pb_encode.h>
#include <nanopb/pb_decode.h>
// Protocol buffer messages
#include <proto/description.pb.h> // description.proto
#include <proto/signal.pb.h>      // signal.proto

// External libraries
#include "Adafruit_MQTT.h"
#include "Arduino.h"
#include <arduino-timer.h>

// Internal libraries
#include "BlinkaConnect_Boards.h"

// Reserved BlinkaConnect topics
#define TOPIC_DESCRIPTION "/description" ///< Device description topic
#define TOPIC_SIGNALS     "/signals/"     ///< Device signals topic

#define BC_PRINTER Serial ///< Where debug messages will be printed
#define BC_DEBUG
// Define actual debug output functions when necessary.
#ifdef BC_DEBUG
#define BC_DEBUG_PRINT(...)                                                   \
  { BC_PRINTER.print(__VA_ARGS__); } ///< Prints debug output.
#define BC_DEBUG_PRINTLN(...)                                                 \
  { BC_PRINTER.println(__VA_ARGS__); } ///< Prints line from debug output.
#else
#define BC_DEBUG_PRINT(...)                                                   \
  {} ///< Prints debug output
#define BC_DEBUG_PRINTLN(...)                                                 \
  {} ///< Prints line from debug output.
#endif

// Adafruit IO Status States
typedef enum {
  BC_IDLE = 0,               // Waiting for connection establishement
  BC_NET_DISCONNECTED = 1,   // Network disconnected
  BC_DISCONNECTED = 2,       // Disconnected from Adafruit IO
  BC_FINGERPRINT_UNKOWN = 3, // Unknown BC_SSL_FINGERPRINT

  BC_NET_CONNECT_FAILED = 10,  // Failed to connect to network
  BC_CONNECT_FAILED = 11,      // Failed to connect to Adafruit IO
  BC_FINGERPRINT_INVALID = 12, // Unknown BC_SSL_FINGERPRINT
  BC_AUTH_FAILED = 13, // Invalid Adafruit IO login credentials provided.
  BC_SSID_INVALID =
      14, // SSID is "" or otherwise invalid, connection not attempted

  BC_NET_CONNECTED = 20,           // Connected to Adafruit IO
  BC_CONNECTED = 21,               // Connected to network
  BC_CONNECTED_INSECURE = 22,      // Insecurely (non-SSL) connected to network
  BC_FINGERPRINT_UNSUPPORTED = 23, // Unsupported BC_SSL_FINGERPRINT
  BC_FINGERPRINT_VALID = 24,       // Valid BC_SSL_FINGERPRINT

  BC_BOARD_DESC_INVALID = 25       // Unable to send board description
} bc_status_t;

// BlinkaConnect board definition status
typedef enum {
    BC_BOARD_DEF_IDLE,
    BC_BOARD_DEF_SEND_FAILED,
    BC_BOARD_DEF_SENT,
    BC_BOARD_DEF_OK,
    BC_BOARD_DEF_INAVLID_VID,
    BC_BOARD_DEF_INVALID_PID,
    BC_BOARD_DEF_UNSPECIFIED
} bc_board_status_t;



class BlinkaConnect {
    
    public:
        BlinkaConnect();
        virtual ~BlinkaConnect();

        void connect();
        virtual void _connect() = 0;

        void disconnect();
        virtual void _disconnect() = 0;

        const __FlashStringHelper *statusText();
        virtual bc_status_t networkStatus() = 0;
        bc_status_t status();
        bc_status_t mqttStatus();
        bc_board_status_t getBoardStatus();

        bool sendBoardDescription();
        bool sendGetHardwareDescription();

        bc_status_t checkNetworkConnection(uint32_t timeStart);
        bc_status_t checkMQTTConnection(uint32_t timeStart);
        void ping();
        bc_status_t run();

        bool decodeSignalMessage();
        bool executeSignalMessageEvent();
        bool pinConfig();
        bool pinEvent();
        bool sendPinEvent();

        static bool cbDigitalRead(char *pinName);
        static void cbSignalTopic(char *data, uint16_t len);

        static uint16_t bufSize;
        static uint8_t _buffer[128]; /*!< Shared buffer to save callback payload */
        uint8_t _buffer_state[128]; /*!< Holds previous contents of static _buffer */
        // Protobuf structs
        signal_v1_CreateSignalRequest signalMessage = signal_v1_CreateSignalRequest_init_zero;
        // Protobuf helpers
        bool encode_unionmessage(pb_ostream_t *stream, const pb_msgdesc_t *messagetype, void *message);

        struct pinInfo {
            char *pinName;
            char *PinNameFull;
            int pinValue;
            int prvPinValue; // holds prv. pin state
        };
        static pinInfo bc_pinInfo;

        static Timer<16, millis, char *> t_timer;

    private:
        void _init();

    protected:
        bc_status_t _status = BC_IDLE; /*!< Adafruit IO connection status */
        uint32_t _last_mqtt_connect = 0; /*!< Previous time when client connected to
                                                Adafruit IO, in milliseconds */
        uint32_t _prv_ping = 0;

        Adafruit_MQTT *_mqtt;                         /*!< Reference to Adafruit_MQTT, _mqtt. */

        // Internal Server
        const char *_mqtt_broker = "2.tcp.ngrok.io"; /*!< MQTT Broker URL */
        uint16_t _mqtt_port = 14174;                 /*!< MQTT Broker URL */

        // Production Server
        //const char *_mqtt_broker = "3.87.30.189"; /*!< MQTT Broker URL */
        //uint16_t _mqtt_port = 1883;                           /*!< MQTT Broker URL */

        const char *_deviceId; /*!< Adafruit IO+ device identifier string */
        uint16_t _hw_vid;   /*!< USB vendor identifer */
        uint16_t _hw_pid;   /*!< USB product identifier */

        // MQTT topics
        char *_topic_description; /*!< MQTT topic for the device description  */
        char *_topic_description_status; /*!< MQTT subtopic carrying the description status resp. from the broker */
        char *_topic_signals_in;     /*!< Device -> Server communication channel */
        char *_topic_signals_out;     /*!< Server -> Device communication channel */

        Adafruit_MQTT_Subscribe *_topic_description_sub;
        Adafruit_MQTT_Publish *_topic_signals_in_pub;
        Adafruit_MQTT_Subscribe *_topic_signals_out_sub;
        Adafruit_MQTT_Subscribe *_subscription;

        static char _value[45]; /*!< Data to send back to BlinkaConnect, max. IO data len */
        static char _prv_value[45]; /*!< Data to send back to BlinkaConnect, max. IO data len */
};

#endif // ADAFRUIT_BLINKACONNECT_H