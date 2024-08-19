#ifndef WS_HELPER_STATUS_H
#define WS_HELPER_STATUS_H

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

#endif // WS_HELPER_STATUS_H