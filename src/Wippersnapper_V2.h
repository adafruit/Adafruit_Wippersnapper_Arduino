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

// Cpp STD
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
#include "protos/signal.pb.h"

// External libraries
#include "Adafruit_MQTT.h"      // MQTT Client
#include "Adafruit_SleepyDog.h" // Watchdog
#include "Arduino.h"            // Wiring
#include <SPI.h>                // SPI

// Wippersnapper API Helpers
#include "Wippersnapper_Boards.h"
#include "components/statusLED/Wippersnapper_StatusLED.h"
#include "helpers/ws_helper_status.h"
#ifdef ARDUINO_ARCH_ESP32
#include "helpers/ws_helper_esp.h"
#endif

// Components (API v2)
#include "components/checkin/model.h"
#include "components/sensor/model.h"
#include "components/digitalIO/controller.h"
#include "components/analogio/controller.h"


// Components (API v1)
#include "components/analogIO/Wippersnapper_AnalogIO.h"
#include "components/ds18x20/ws_ds18x20.h"
#include "components/i2c/WipperSnapper_I2C.h"
#include "components/pixels/ws_pixels.h"
#include "components/pwm/ws_pwm.h"
#include "components/servo/ws_servo.h"
#include "components/uart/ws_uart.h"
#ifdef ARDUINO_ARCH_ESP32
#include "components/ledc/ws_ledc.h"
#endif
// Display
#ifdef USE_DISPLAY
#include "display/ws_display_driver.h"
#include "display/ws_display_ui_helper.h"
#endif



#include "provisioning/ConfigJson.h"
#if defined(USE_TINYUSB)
#include "provisioning/tinyusb/Wippersnapper_FS_V2.h"
#endif
#if defined(USE_LITTLEFS)
#include "provisioning/littlefs/WipperSnapper_LittleFS.h"
#endif

#define WS_VERSION                                                             \
  "2.0.0-alpha.1" ///< WipperSnapper app. version (semver-formatted)

#define WS_WDT_TIMEOUT 60000       ///< WDT timeout
#define WS_MAX_ALT_WIFI_NETWORKS 3 ///< Maximum number of alternative networks
/* MQTT Configuration */
#define WS_KEEPALIVE_INTERVAL_MS                                               \
  5000 ///< Session keepalive interval time, in milliseconds

#define WS_MQTT_MAX_PAYLOAD_SIZE                                               \
  512 ///< MAXIMUM expected payload size, in bytes

// Forward declarations (API v1)
class Wippersnapper_AnalogIO;
class Wippersnapper_FS_V2;
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

// Forward declarations (API v2)
class CheckinModel;
class SensorModel;
class DigitalIOController;
class AnalogIOController;

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

  void provisionV2();

  bool
      lockStatusNeoPixelV2; ///< True if status LED is using the status neopixel
  bool lockStatusDotStarV2; ///< True if status LED is using the status dotstar
  bool lockStatusLEDV2;     ///< True if status LED is using the built-in LED
  float status_pixel_brightnessV2 =
      STATUS_PIXEL_BRIGHTNESS_DEFAULT; ///< Global status pixel's brightness
                                       ///< (from 0.0 to 1.0)

  virtual void set_user_keyV2();
  virtual void set_ssid_passV2(const char *ssid, const char *ssidPassword);
  virtual void set_ssid_passV2();
  virtual bool check_valid_ssidV2();

  virtual void _connectV2();
  virtual void _disconnectV2();
  void connectV2();
  void disconnectV2();

  virtual void getMacAddrV2();
  virtual int32_t getRSSIV2();
  virtual void setupMQTTClientV2(const char *clientID);

  virtual ws_status_t networkStatusV2();

  // Generators for device UID and MQTT topics
  bool generateDeviceUIDV2();
  bool generateWSTopicsV2();

  // High-level MQTT Publish
  bool PublishSignal(pb_size_t which_payload, void *payload);

  // Checkin API
  bool CreateCheckinRequest();
  void PollCheckinResponse();

  // run() loop
  ws_status_t runV2();
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

  // I2C
  // TODO: Audit all of this!
  std::vector<WipperSnapper_Component_I2C *>
      i2cComponentsV2; ///< Vector containing all I2C components
  WipperSnapper_Component_I2C *_i2cPort0V2 =
      NULL; ///< WipperSnapper I2C Component for I2C port #0
  WipperSnapper_Component_I2C *_i2cPort1V2 =
      NULL; ///< WipperSnapper I2C Component for I2C port #1
  bool _isI2CPort0InitV2 =
      false; ///< True if I2C port 0 has been initialized, False otherwise.
  bool _isI2CPort1InitV2 =
      false; ///< True if I2C port 1 has been initialized, False otherwise.

  // TODO: Do we need this?
  ws_board_status_t _boardStatusV2 =
      WS_BOARD_DEF_IDLE; ///< Hardware's registration status

  // TODO: We really should look at making these static definitions, not dynamic
  // to free up space on the heap
  Wippersnapper_AnalogIO *_analogIOV2; ///< Instance of analog io class
  Wippersnapper_FS_V2 *_fileSystemV2;  ///< Instance of Filesystem (native USB)
  WipperSnapper_LittleFS
      *_littleFSV2; ///< Instance of LittleFS Filesystem (non-native USB)
#ifdef USE_DISPLAY
  ws_display_driver *_displayV2 = nullptr; ///< Instance of display driver class
  ws_display_ui_helper *_ui_helperV2 =
      nullptr; ///< Instance of display UI helper class
#endif
  ws_pixels *_ws_pixelsComponentV2; ///< ptr to instance of ws_pixels class
  ws_pwm *_pwmComponentV2;          ///< Instance of pwm class
  ws_servo *_servoComponentV2;      ///< Instance of servo class
  ws_ds18x20 *_ds18x20ComponentV2;  ///< Instance of DS18x20 class
  ws_uart *_uartComponentV2;        ///< Instance of UART class

  // API v2 Components
  CheckinModel *CheckInModel;       ///< Instance of CheckinModel class
  SensorModel  *sensorModel;        ///< Instance of SensorModel class
  DigitalIOController
      *digital_io_controller; ///< Instance of DigitalIO controller class
  AnalogIOController *analogio_controller; ///< Instance of AnalogIO controller

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
#ifdef ARDUINO_ARCH_ESP32
  ws_ledc *_ledcV2 = nullptr; ///< Pointer to LEDC object
#endif
  bool got_checkin_response; ///< True if a checkin response was received, False otherwise.

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

  wippersnapper_signal_v1_CreateSignalRequest
      _outgoingSignalMsgV2; /*!< Outgoing signal message from device */
};
extern Wippersnapper_V2 WsV2; ///< Global member variable for callbacks

#endif // WIPPERSNAPPER_V2_H
