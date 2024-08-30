/*!
 * @file Wippersnapper_v2.cpp
 *
 * @mainpage Adafruit Wippersnapper Wrapper
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's Wippersnapper wrapper for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit IO+ Wippersnapper IoT platform.
 *
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on <a
 * href="https://github.com/adafruit/Adafruit_Sensor"> Adafruit_Sensor</a> being
 * present on your system. Please make sure you have installed the latest
 * version before using this library.
 *
 * @section author Author
 *
 * Copyright (c) Brent Rubell 2020-2023 for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper_V2.h"

Wippersnapper_V2 WsV2;

Wippersnapper_V2::Wippersnapper_V2() {
  // TODO: Scope out how much of this we can remove
  // and what should be here (if we are wrong!)
  _mqttV2 = 0; // MQTT Client object

  // Reserved MQTT Topics
  _topicError = 0;
  _topicThrottle = 0;
  _subscribeError = 0;
  _subscribeThrottle = 0;

  // Init. component classes
  // LEDC (ESP32-ONLY)
#ifdef ARDUINO_ARCH_ESP32
  WsV2._ledcV2 = new ws_ledc();
#endif

  // PWM (Arch-specific implementations)
#ifdef ARDUINO_ARCH_ESP32
  WsV2._pwmComponentV2 = new ws_pwm(WsV2._ledcV2);
#else
  WsV2._pwmComponentV2 = new ws_pwm();
#endif

  // Servo
  WsV2._servoComponentV2 = new ws_servo();

  // UART
  WsV2._uartComponentV2 = new ws_uart();

  // DallasSemi (OneWire)
  WsV2._ds18x20ComponentV2 = new ws_ds18x20();
};

/**************************************************************************/
/*!
    @brief    Wippersnapper_V2 destructor
*/
/**************************************************************************/
Wippersnapper_V2::~Wippersnapper_V2() {}

/**************************************************************************/
/*!
    @brief    Provisions a WipperSnapper device with its network
              configuration and Adafruit IO credentials.
*/
/**************************************************************************/
void Wippersnapper_V2::provisionV2() {
  // Obtain device's MAC address
  getMacAddrV2();

  // Initialize the status LED for signaling FS errors
  initStatusLED();

// Initialize the filesystem
#ifdef USE_TINYUSB
  _fileSystemV2 = new Wippersnapper_FS_V2();
#elif defined(USE_LITTLEFS)
  _littleFS = new WipperSnapper_LittleFS();
#endif

#ifdef USE_DISPLAY
  // Initialize the display
  displayConfig config;
  WsV2._fileSystemV2->parseDisplayConfig(config);
  WsV2._display = new ws_display_driver(config);
  // Begin display
  if (!WsV2._display->begin()) {
    WS_DEBUG_PRINTLN("Unable to enable display driver and LVGL");
    haltErrorV2("Unable to enable display driver, please check the json "
                "configuration!");
  }

  WsV2._display->enableLogging();
  releaseStatusLED(); // don't use status LED if we are using the display
  // UI Setup
  WsV2._ui_helper = new ws_display_ui_helper(WsV2._display);
  WsV2._ui_helper->set_bg_black();
  WsV2._ui_helper->show_scr_load();
  WsV2._ui_helper->set_label_status("Validating Credentials...");
#endif

  // Parse secrets.json file
#ifdef USE_TINYUSB
  _fileSystemV2->parseSecrets();
#elif defined(USE_LITTLEFS)
  _littleFS->parseSecrets();
#else
  check_valid_ssidV2(); // non-fs-backed, sets global credentials within network
                        // iface
#endif
  // Set the status pixel's brightness
  setStatusLEDBrightness(WsV2._configV2.status_pixel_brightness);
  // Set device's wireless credentials
  set_ssid_passV2();

#ifdef USE_DISPLAY
  WsV2._ui_helper->set_label_status("");
  WsV2._ui_helper->set_load_bar_icon_complete(loadBarIconFile);
#endif
}

/**************************************************************************/
/*!
    @brief    Disconnects from Adafruit IO+ Wippersnapper_V2.
*/
/**************************************************************************/
void Wippersnapper_V2::disconnectV2() { _disconnectV2(); }

// Concrete class definition for abstract classes

/****************************************************************************/
/*!
    @brief    Connects to wireless network.
*/
/****************************************************************************/
void Wippersnapper_V2::_connectV2() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::_connectV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Disconnect Wippersnapper MQTT session and network.
*/
/****************************************************************************/
void Wippersnapper_V2::_disconnectV2() {
  WS_DEBUG_PRINTLN("WIppersnapper_V2::_disconnectV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets the network interface's unique identifer, typically the
              MAC address.
*/
/****************************************************************************/
void Wippersnapper_V2::getMacAddrV2() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::getMacAddrV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Gets the network's RSSI.
    @return   int32_t RSSI value, 0 to 255, in dB
*/
/****************************************************************************/
int32_t Wippersnapper_V2::getRSSIV2() {
  WS_DEBUG_PRINTLN("Wiippersnapper_V2::getRSSIV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return 0;
}

/****************************************************************************/
/*!
    @brief    Sets up the MQTT client session.
    @param    clientID
              A unique client identifier string.
*/
/****************************************************************************/
void Wippersnapper_V2::setupMQTTClientV2(const char * /*clientID*/) {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::setupMQTTClientV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Returns the network's connection status
    @returns  Network status as ws_status_t.
*/
/****************************************************************************/
ws_status_t Wippersnapper_V2::networkStatusV2() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::networkStatusV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return WS_IDLE;
}

/****************************************************************************/
/*!
    @brief    Sets the device's wireless network credentials.
    @param    ssid
              Your wireless network's SSID
    @param    ssidPassword
              Your wireless network's password.
*/
/****************************************************************************/
void Wippersnapper_V2::set_ssid_passV2(const char * /*ssid*/,
                                       const char * /*ssidPassword*/) {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::set_ssid_passV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets the device's wireless network credentials from the
              secrets.json configuration file.
*/
/****************************************************************************/
void Wippersnapper_V2::set_ssid_passV2() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::set_ssid_passV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/***********************************************************/
/*!
@brief   Performs a scan of local WiFi networks.
@returns True if `_network_ssid` is found, False otherwise.
*/
/***********************************************************/
bool Wippersnapper_V2::check_valid_ssidV2() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::check_valid_ssidV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return false;
}

/****************************************************************************/
/*!
    @brief    Configures the device's Adafruit IO credentials. This method
              should be used only if filesystem-backed provisioning is
              not avaliable.
*/
/****************************************************************************/
void Wippersnapper_V2::set_user_keyV2() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::set_user_keyV2");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Handles a Checkin Response message and initializes the
              device's GPIO classes.
    @param    stream
              Incoming data stream from buffer.
    @returns  True if Checkin Response decoded and parsed successfully,
              False otherwise.
*/
/****************************************************************************/
bool handleCheckinResponse(pb_istream_t *stream) {
  // Decode the Checkin Response message
  if (!WsV2.CheckInModel->DecodeCheckinResponse(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode Checkin Response message");
    return false;
  }

  // Parse the response message
  WsV2.CheckInModel->ParseCheckinResponse();

  // Validate the checkin response message
  if (WsV2.CheckInModel->getCheckinResponse() !=
      wippersnapper_checkin_CheckinResponse_Response_RESPONSE_OK) {
    WS_DEBUG_PRINTLN("ERROR: CheckinResponse not RESPONSE_OK, backing out!");
    return false;
  }

  // Configure GPIO classes based on checkin response message
  WsV2._digitalGPIOV2 =
      new Wippersnapper_DigitalGPIO(WsV2.CheckInModel->getTotalGPIOPins());
  WsV2._analogIOV2 =
      new Wippersnapper_AnalogIO(WsV2.CheckInModel->getTotalAnalogPins(),
                                 WsV2.CheckInModel->getReferenceVoltage());

  // set glob flag so we don't keep the polling loop open
  WsV2.got_checkin_response = true;
  return true;
}

// Decoders //

/******************************************************************************************/
/*!
    @brief    Decodes a BrokerToDevice message and executes the asscoiated
   callback.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeBrokerToDevice(pb_istream_t *stream, const pb_field_t *field,
                            void **arg) {
  WS_DEBUG_PRINTLN("cbDecodeBrokerToDevice()");
  (void)arg; // marking unused parameters to avoid compiler warning

  switch (field->tag) {
  case wippersnapper_signal_BrokerToDevice_checkin_response_tag:
    WS_DEBUG_PRINTLN("GOT: Checkin Response");
    if (!handleCheckinResponse(stream)) {
      WS_DEBUG_PRINTLN("Failure handling Checkin Response!");
      return false;
    }
    break;
  default:
    WS_DEBUG_PRINTLN("ERROR: BrokerToDevice message type not found!");
    return false;
  }

  // once this is returned, pb_dec_submessage()
  // decodes the submessage contents.
  return true;
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /ws-b2d/ "signal topic".
    @param    data
                Data (payload) from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbBrokerToDevice(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("=> New B2D message!");
  wippersnapper_signal_BrokerToDevice msg_signal =
      wippersnapper_signal_BrokerToDevice_init_default;

  // Configure the payload callback
  msg_signal.cb_payload.funcs.decode = cbDecodeBrokerToDevice;

  // Decode msg_signal
  WS_DEBUG_PRINTLN("Creating input stream...");
  pb_istream_t istream = pb_istream_from_buffer((uint8_t *)data, len);
  WS_DEBUG_PRINTLN("Decoding BrokerToDevice message...");
  if (!pb_decode(&istream, wippersnapper_signal_BrokerToDevice_fields,
                 &msg_signal)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode BrokerToDevice message!");
    return;
  }
  WS_DEBUG_PRINTLN("Decoded BrokerToDevice message!");
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /error special topic.
    @param    errorData
                Data from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbErrorTopicV2(char *errorData, uint16_t len) {
  (void)len; // marking unused parameter to avoid compiler warning
  WS_DEBUG_PRINT("IO Ban Error: ");
  WS_DEBUG_PRINTLN(errorData);
  // Disconnect client from broker
  WS_DEBUG_PRINT("Disconnecting from MQTT..");
  if (!WsV2._mqttV2->disconnect()) {
    WS_DEBUG_PRINTLN("ERROR: Unable to disconnect from MQTT broker!");
  }

#ifdef USE_DISPLAY
  WsV2._ui_helper->show_scr_error("IO Ban Error", errorData);
#endif

  // WDT reset
  WsV2.haltErrorV2("IO MQTT Ban Error");
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /throttle special topic. Delays until
                throttle is released.
    @param    throttleData
                Throttle message from Adafruit IO.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbThrottleTopicV2(char *throttleData, uint16_t len) {
  (void)len; // marking unused parameter to avoid compiler warning
  WS_DEBUG_PRINT("IO Throttle Error: ");
  WS_DEBUG_PRINTLN(throttleData);
  char *throttleMessage;
  // Parse out # of seconds from message buffer
  throttleMessage = strtok(throttleData, ",");
  throttleMessage = strtok(NULL, " ");
  // Convert from seconds to to millis
  int throttleDuration = atoi(throttleMessage) * 1000;

  WS_DEBUG_PRINT("Device is throttled for ");
  WS_DEBUG_PRINT(throttleDuration);
  WS_DEBUG_PRINTLN("ms and blocking command execution.");

#ifdef USE_DISPLAY
  char buffer[100];
  snprintf(
      buffer, 100,
      "[IO ERROR] Device is throttled for %d mS and blocking execution..\n.",
      throttleDuration);
  WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  // If throttle duration is less than the keepalive interval, delay for the
  // full keepalive interval
  if (throttleDuration < WS_KEEPALIVE_INTERVAL_MS) {
    delay(WS_KEEPALIVE_INTERVAL_MS);
  } else {
    // round to nearest millis to prevent delaying for less time than req'd.
    float throttleLoops = ceil(throttleDuration / WS_KEEPALIVE_INTERVAL_MS);
    // block the run() loop
    while (throttleLoops > 0) {
      delay(WS_KEEPALIVE_INTERVAL_MS);
      WsV2.feedWDTV2();
      WsV2._mqttV2->ping();
      throttleLoops--;
    }
  }
  WS_DEBUG_PRINTLN("Device is un-throttled, resumed command execution");
#ifdef USE_DISPLAY
  WsV2._ui_helper->add_text_to_terminal(
      "[IO] Device is un-throttled, resuming...\n");
#endif
}

/**************************************************************************/
/*!
    @brief    Attempts to generate unique device identifier.
    @returns  True if device identifier generated successfully,
              False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_V2::generateDeviceUIDV2() {
  // Generate device unique identifier
  // Set machine_name
  WsV2._boardIdV2 = BOARD_ID;
  // Move the top 3 bytes from the UID
  for (int i = 5; i > 2; i--) {
    WsV2._macAddrV2[6 - 1 - i] = WsV2._macAddrV2[i];
  }
  snprintf(WsV2.sUIDV2, sizeof(WsV2.sUIDV2), "%02d%02d%02d", WsV2._macAddrV2[0],
           WsV2._macAddrV2[1], WsV2._macAddrV2[2]);
  // Conversion to match integer UID sent by createMsgCheckinRequest()
  itoa(atoi(WsV2.sUIDV2), WsV2.sUIDV2, 10);

  // Calculate the length of device and UID strings
  WS_DEBUG_PRINTLN("Calculating device UID length...");
  size_t lenBoardId = strlen(WsV2._boardIdV2);
  size_t lenUID = strlen(WsV2.sUIDV2);
  size_t lenIOWipper = strlen("io-wipper-");
  size_t lenDeviceUID = lenBoardId + lenUID + lenIOWipper + 1;

  // Attempt to allocate memory for the _device_uid
  WS_DEBUG_PRINTLN("Allocating memory for device UID");
#ifdef USE_PSRAM
  _device_uidV2 = (char *)ps_malloc(sizeof(char) * lenDeviceUID);
#else
  _device_uidV2 = (char *)malloc(sizeof(char) * lenDeviceUID);
#endif

  // Check if memory allocation was successful
  if (_device_uidV2 == NULL) {
    WS_DEBUG_PRINTLN("ERROR: Unable to create device uid, Malloc failure");
    return false;
  }

  // Create the device identifier
  snprintf(_device_uidV2, lenDeviceUID, "io-wipper-%s%s", WsV2._boardIdV2,
           WsV2.sUIDV2);
  WS_DEBUG_PRINT("Device UID: ");
  WS_DEBUG_PRINTLN(_device_uidV2);

  return true;
}

/**************************************************************************/
/*!
    @brief    Generates device-specific Wippersnapper control topics and
              subscribes to them.
    @returns  True if memory for control topics allocated successfully,
                False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_V2::generateWSTopicsV2() {
  WS_DEBUG_PRINTLN("Pre-calculating topic lengths...");
  // Calculate length of strings that are are dynamic within the secrets file
  size_t lenUser = strlen(WsV2._configV2.aio_user);
  size_t lenBoardId = strlen(_device_uidV2);
  // Calculate length of static strings
  size_t lenTopicX2x = strlen("/ws-x2x/");
  size_t lenTopicErrorStr = strlen("/errors/");
  size_t lenTopicThrottleStr = strlen("/throttle/");
  // Calculate length of complete topic strings
  // NOTE: We are using "+2" to account for the null terminator and the "/" at
  // the end of the topic
  size_t lenTopicB2d = lenUser + lenTopicX2x + lenBoardId + 2;
  size_t lenTopicD2b = lenUser + lenTopicX2x + lenBoardId + 2;
  size_t lenTopicError = lenUser + lenTopicErrorStr + 2;
  size_t lenTopicThrottle = lenUser + lenTopicThrottleStr + 2;

  // Attempt to allocate memory for the broker-to-device topic
#ifdef USE_PSRAM
  WsV2._topicB2d = (char *)ps_malloc(sizeof(char) * lenTopicB2d);
#else
  WsV2._topicB2d = (char *)malloc(sizeof(char) * lenTopicB2d);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicB2d == NULL)
    return false;
  // Build the broker-to-device topic
  snprintf(WsV2._topicB2d, lenTopicB2d, "%s/ws-b2d/%s/",
           WsV2._configV2.aio_user, _device_uidV2);
  WS_DEBUG_PRINT("Broker-to-device topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicB2d);
  // Subscribe to broker-to-device topic
  _subscribeB2d = new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicB2d, 1);
  WsV2._mqttV2->subscribe(_subscribeB2d);
  _subscribeB2d->setCallback(cbBrokerToDevice);

  // Create global device to broker topic
  // Attempt to allocate memory for the broker-to-device topic
#ifdef USE_PSRAM
  WsV2._topicD2b = (char *)ps_malloc(sizeof(char) * lenTopicD2b);
#else
  WsV2._topicD2b = (char *)malloc(sizeof(char) * lenTopicD2b);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicD2b == NULL)
    return false;
  // Build the broker-to-device topic
  snprintf(WsV2._topicD2b, lenTopicD2b, "%s/ws-d2b/%s/",
           WsV2._configV2.aio_user, _device_uidV2);
  WS_DEBUG_PRINT("Device-to-broker topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicD2b);

  // Attempt to allocate memory for the error topic
#ifdef USE_PSRAM
  WsV2._topicError = (char *)ps_malloc(sizeof(char) * lenTopicError);
#else
  WsV2._topicError = (char *)malloc(sizeof(char) * lenTopicError);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicError == NULL)
    return false;
  // Build the error topic
  snprintf(WsV2._topicError, lenTopicError, "%s/%s/", WsV2._configV2.aio_user,
           "errors");
  WS_DEBUG_PRINT("Error topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicError);
  // Subscribe to the error topic
  _subscribeError = new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicError);
  WsV2._mqttV2->subscribe(_subscribeError);
  // TODO: Implement the error topic callback
  _subscribeError->setCallback(cbErrorTopicV2);

// Attempt to allocate memory for the error topic
#ifdef USE_PSRAM
  WsV2._topicThrottle = (char *)ps_malloc(sizeof(char) * lenTopicThrottle);
#else
  WsV2._topicThrottle = (char *)malloc(sizeof(char) * lenTopicThrottle);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicThrottle == NULL)
    return false;
  // Build the throttle topic
  snprintf(WsV2._topicThrottle, lenTopicThrottle, "%s/%s/",
           WsV2._configV2.aio_user, "throttle");
  WS_DEBUG_PRINT("Throttle topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicThrottle);
  // Subscribe to throttle topic
  _subscribeThrottle =
      new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicThrottle);
  WsV2._mqttV2->subscribe(_subscribeThrottle);
  _subscribeThrottle->setCallback(cbThrottleTopicV2);

  return true;
}

/**************************************************************************/
/*!
    @brief    Writes an error message to the serial and the filesystem,
                blinks WS_LED_STATUS_ERROR_RUNTIME pattern and hangs.
    @param    error
              The error message to write to the serial and filesystem.
*/
/**************************************************************************/
void Wippersnapper_V2::errorWriteHangV2(String error) {
  // Print error
  WS_DEBUG_PRINTLN(error);
#ifdef USE_TINYUSB
  _fileSystemV2->writeToBootOut(error.c_str());
  TinyUSBDevice.attach();
  delay(500);
#endif
  // Signal and hang forever
  while (1) {
    WS_DEBUG_PRINTLN("ERROR: Halted execution");
    WS_DEBUG_PRINTLN(error.c_str());
    WsV2.feedWDTV2();
    statusLEDBlink(WS_LED_STATUS_ERROR_RUNTIME);
    delay(1000);
  }
}

/**************************************************************************/
/*!
    @brief    Checks network and MQTT connectivity. Handles network
              re-connection and mqtt re-establishment.
*/
/**************************************************************************/
void Wippersnapper_V2::runNetFSMV2() {
  WsV2.feedWDTV2();
  // Initial state
  fsm_net_t fsmNetwork;
  fsmNetwork = FSM_NET_CHECK_MQTT;
  int maxAttempts;
  while (fsmNetwork != FSM_NET_CONNECTED) {
    switch (fsmNetwork) {
    case FSM_NET_CHECK_MQTT:
      if (WsV2._mqttV2->connected()) {
        // WS_DEBUG_PRINTLN("Connected to Adafruit IO!");
        fsmNetwork = FSM_NET_CONNECTED;
        return;
      }
      fsmNetwork = FSM_NET_CHECK_NETWORK;
      break;
    case FSM_NET_CHECK_NETWORK:
      if (networkStatusV2() == WS_NET_CONNECTED) {
        WS_DEBUG_PRINTLN("Connected to WiFi!");
#ifdef USE_DISPLAY
        if (WsV2._ui_helper->getLoadingState())
          WsV2._ui_helper->set_load_bar_icon_complete(loadBarIconWifi);
#endif
        fsmNetwork = FSM_NET_ESTABLISH_MQTT;
        break;
      }
      fsmNetwork = FSM_NET_ESTABLISH_NETWORK;
      break;
    case FSM_NET_ESTABLISH_NETWORK:
      WS_DEBUG_PRINTLN("Establishing network connection...");
      WS_PRINTER.flush();
#ifdef USE_DISPLAY
      if (WsV2._ui_helper->getLoadingState())
        WsV2._ui_helper->set_label_status("Connecting to WiFi...");
#endif
      // Perform a WiFi scan and check if SSID within
      // secrets.json is within the scanned SSIDs
      WS_DEBUG_PRINT("Performing a WiFi scan for SSID...");
      if (!check_valid_ssidV2()) {
#ifdef USE_DISPLAY
        WsV2._ui_helper->show_scr_error("ERROR",
                                        "Unable to find WiFi network listed in "
                                        "the secrets file. Rebooting soon...");
#endif
        haltErrorV2("ERROR: Unable to find WiFi network, rebooting soon...",
                    WS_LED_STATUS_WIFI_CONNECTING);
      }
      // Attempt to connect to wireless network
      maxAttempts = 5;
      while (maxAttempts > 0) {
        // blink before we connect
        statusLEDBlink(WS_LED_STATUS_WIFI_CONNECTING);
        feedWDTV2();
        // attempt to connect
        WS_DEBUG_PRINT("Connecting to WiFi (attempt #");
        WS_DEBUG_PRINT(5 - maxAttempts);
        WS_DEBUG_PRINTLN(")");
        WS_PRINTER.flush();
        feedWDTV2();
        _connectV2();
        feedWDTV2();
        // did we connect?
        if (networkStatusV2() == WS_NET_CONNECTED)
          break;
        maxAttempts--;
      }
      // Validate connection
      if (networkStatusV2() != WS_NET_CONNECTED) {
        WS_DEBUG_PRINTLN("ERROR: Unable to connect to WiFi!");
#ifdef USE_DISPLAY
        WsV2._ui_helper->show_scr_error(
            "CONNECTION ERROR",
            "Unable to connect to WiFi Network. Please check that you entered "
            "the WiFi credentials correctly. Rebooting in 5 seconds...");
#endif
        haltErrorV2("ERROR: Unable to connect to WiFi, rebooting soon...",
                    WS_LED_STATUS_WIFI_CONNECTING);
      }

      fsmNetwork = FSM_NET_CHECK_NETWORK;
      break;
    case FSM_NET_ESTABLISH_MQTT:
#ifdef USE_DISPLAY
      if (WsV2._ui_helper->getLoadingState())
        WsV2._ui_helper->set_label_status("Connecting to IO...");
#endif
      WsV2._mqttV2->setKeepAliveInterval(WS_KEEPALIVE_INTERVAL_MS / 1000);
      // Attempt to connect
      maxAttempts = 5;
      while (maxAttempts > 0) {
        WS_DEBUG_PRINT("Connecting to AIO MQTT (attempt #");
        WS_DEBUG_PRINT(5 - maxAttempts);
        WS_DEBUG_PRINTLN(")");
        WS_PRINTER.flush();
        WS_DEBUG_PRINT("WiFi Status: ");
        WS_DEBUG_PRINTLN(networkStatusV2());
        WS_PRINTER.flush();
        feedWDTV2();
        statusLEDBlink(WS_LED_STATUS_MQTT_CONNECTING);
        feedWDTV2();
        int8_t mqttRC = WsV2._mqttV2->connect();
        feedWDTV2();
        if (mqttRC == WS_MQTT_CONNECTED) {
          fsmNetwork = FSM_NET_CHECK_MQTT;
          break;
        }
        WS_DEBUG_PRINT("MQTT Connection Error: ");
        WS_DEBUG_PRINTLN(mqttRC);
        WS_DEBUG_PRINTLN(WsV2._mqttV2->connectErrorString(mqttRC));
        WS_DEBUG_PRINTLN(
            "Unable to connect to Adafruit IO MQTT, retrying in 3 seconds...");
        delay(3000);
        maxAttempts--;
      }
      if (fsmNetwork != FSM_NET_CHECK_MQTT) {
#ifdef USE_DISPLAY
        WsV2._ui_helper->show_scr_error(
            "CONNECTION ERROR",
            "Unable to connect to Adafruit.io. If you are repeatedly having "
            "this issue, please check that your IO Username and IO Key are set "
            "correctly in the secrets file. This device will reboot in 5 "
            "seconds...");
#endif
        haltErrorV2(
            "ERROR: Unable to connect to Adafruit.IO MQTT, rebooting soon...",
            WS_LED_STATUS_MQTT_CONNECTING);
      }
      break;
    default:
      break;
    }
  }
}

/**************************************************************************/
/*!
    @brief    Prints an error to the serial and halts the hardware until
              the WDT bites.
    @param    error
              The desired error to print to serial.
    @param    ledStatusColor
              The desired color to blink.
*/
/**************************************************************************/
void Wippersnapper_V2::haltErrorV2(String error,
                                   ws_led_status_t ledStatusColor) {
  for (;;) {
    WS_DEBUG_PRINT("ERROR [WDT RESET]: ");
    WS_DEBUG_PRINTLN(error);
    // let the WDT fail out and reset!
    statusLEDSolid(ledStatusColor);
#ifndef ARDUINO_ARCH_ESP8266
    delay(1000);
#else
    // Calls to delay() and yield() feed the ESP8266's
    // hardware and software watchdog timers, delayMicroseconds does not.
    delayMicroseconds(1000000);
#endif
  }
}

bool Wippersnapper_V2::PublishSignal(pb_size_t which_payload, void *payload) {
  size_t szMessageBuf;
  wippersnapper_signal_DeviceToBroker MsgSignal =
      wippersnapper_signal_DeviceToBroker_init_default;
  // Fill generic signal payload with the payload from the args.
  WS_DEBUG_PRINT("Signal Payload Type: ");
  switch (which_payload) {
  case wippersnapper_signal_DeviceToBroker_checkin_request_tag:
    WS_DEBUG_PRINTLN("Checkin Request");
    MsgSignal.which_payload =
        wippersnapper_signal_DeviceToBroker_checkin_request_tag;
    MsgSignal.payload.checkin_request =
        *(wippersnapper_checkin_CheckinRequest *)payload;
    break;
  default:
    WS_DEBUG_PRINTLN("ERROR: Invalid signal payload type, bailing out!");
    return false;
  }

  // Get the encoded size of the signal message
  if (!pb_get_encoded_size(&szMessageBuf,
                           wippersnapper_signal_DeviceToBroker_fields,
                           &MsgSignal)) {
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to get encoded size of signal message, bailing out!");
    return false;
  }

  // Size the message buffer to fit the encoded signal message
  uint8_t msgBuf[szMessageBuf];

  // Encode the signal message
  WS_DEBUG_PRINT("Encoding signal message...");
  pb_ostream_t stream = pb_ostream_from_buffer(msgBuf, szMessageBuf);
  if (!ws_pb_encode(&stream, wippersnapper_signal_DeviceToBroker_fields,
                    &MsgSignal)) {
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to encode d2b signal message, bailing out!");
    return false;
  }
  WS_DEBUG_PRINTLN("Encoded!")

  //. Check that we are still connected
  runNetFSMV2();
  WsV2.feedWDTV2();

  // Attempt to publish the signal message to the broker
  WS_DEBUG_PRINT("Publishing signal message to broker...");
  if (!WsV2._mqttV2->publish(WsV2._topicD2b, msgBuf, szMessageBuf, 1)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to publish signal message to broker!");
    return false;
  }
  WS_DEBUG_PRINTLN("Published!");

  return true;
}

/**************************************************************************/
/*!
    @brief    Creates, fills, encodes and publishes a checkin request
              message to the broker.
    @returns  True if the Checkin request message published successfully,
              False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_V2::CreateCheckinRequest() {
  WS_DEBUG_PRINTLN("CreateCheckinRequest()");
  pingBrokerV2();

  WS_DEBUG_PRINTLN("Creating new message...");
  WsV2.CheckInModel = new CheckinModel();
  WsV2.CheckInModel->CreateCheckinRequest(WsV2.sUIDV2, WS_VERSION);

  WS_DEBUG_PRINTLN("Encoding message...");
  if (!WsV2.CheckInModel->EncodeCheckinRequest())
    return false;

  WS_DEBUG_PRINT("Message Size: ");
  WS_DEBUG_PRINTLN(WsV2.CheckInModel->CheckinRequestSz);

  WS_DEBUG_PRINT("Publishing Checkin Request...");
  if (!PublishSignal(wippersnapper_signal_DeviceToBroker_checkin_request_tag,
                     &(WsV2.CheckInModel->_CheckinRequest)))
    return false;
  WS_DEBUG_PRINTLN("Published!");

  return true;
}

/**************************************************************************/
/*!
    @brief    Polls for and handles the checkin response
              message from the broker.
*/
/**************************************************************************/
void Wippersnapper_V2::HandleCheckinResponse() {
  WsV2.got_checkin_response = false;
  WS_DEBUG_PRINTLN("Waiting for checkin response...");
  while (!WsV2.got_checkin_response) {
    WsV2.runNetFSMV2();
    WsV2._mqttV2->processPackets(10); // fast poll for incoming packets
  }
  WS_DEBUG_PRINTLN("Completed checkin process!");
}

/**************************************************************************/
/*!
    @brief  Pings the MQTT broker within the keepalive interval
            to keep the connection alive. Blinks the keepalive LED
            every STATUS_LED_KAT_BLINK_TIME milliseconds.
*/
/**************************************************************************/
void Wippersnapper_V2::pingBrokerV2() {
  // ping within keepalive-10% to keep connection open
  if (millis() > (_prv_pingV2 + (WS_KEEPALIVE_INTERVAL_MS -
                                 (WS_KEEPALIVE_INTERVAL_MS * 0.10)))) {
    WS_DEBUG_PRINT("Sending MQTT PING: ");
    if (WsV2._mqttV2->ping()) {
      WS_DEBUG_PRINTLN("SUCCESS!");
    } else {
      WS_DEBUG_PRINTLN("FAILURE! Running network FSM...");
      WsV2._mqttV2->disconnect();
      runNetFSMV2();
    }
    _prv_pingV2 = millis();
    WS_DEBUG_PRINT("WiFi RSSI: ");
    WS_DEBUG_PRINTLN(getRSSIV2());
  }
  // blink status LED every STATUS_LED_KAT_BLINK_TIME millis
  if (millis() > (_prvKATBlinkV2 + STATUS_LED_KAT_BLINK_TIME)) {
    WS_DEBUG_PRINTLN("STATUS LED BLINK KAT");
#ifdef USE_DISPLAY
    WsV2._ui_helper->add_text_to_terminal("[NET] Sent KeepAlive ping!\n");
#endif
    statusLEDBlink(WS_LED_STATUS_KAT);
    _prvKATBlinkV2 = millis();
  }
}

/********************************************************/
/*!
    @brief    Feeds the WDT to prevent hardware reset.
*/
/*******************************************************/
void Wippersnapper_V2::feedWDTV2() { Watchdog.reset(); }

/********************************************************/
/*!
    @brief  Enables the watchdog timer.
    @param  timeoutMS
            The desired amount of time to elapse before
            the WDT executes.
*/
/*******************************************************/
void Wippersnapper_V2::enableWDTV2(int timeoutMS) {
#ifndef ARDUINO_ARCH_RP2040
  Watchdog.disable();
#endif
  if (Watchdog.enable(timeoutMS) == 0) {
    WsV2.haltErrorV2("WDT initialization failure!");
  }
}

/********************************************************/
/*!
    @brief  Process all incoming packets from the
            Adafruit IO MQTT broker. Handles network
            connectivity.
*/
/*******************************************************/
void Wippersnapper_V2::processPacketsV2() {
  // runNetFSMV2(); // NOTE: Removed for now, causes error with virtual
  // _connectV2 method when caused with WsV2 object in another file.
  WsV2.feedWDTV2();
  // Process all incoming packets from Wippersnapper_V2 MQTT Broker
  WsV2._mqttV2->processPackets(10);
}

/**************************************************************************/
/*!
    @brief    Prints information about the WsV2 device to the serial monitor.
*/
/**************************************************************************/
void printDeviceInfoV2() {
  WS_DEBUG_PRINTLN("-------Device Information-------");
  WS_DEBUG_PRINT("Firmware Version: ");
  WS_DEBUG_PRINTLN(WS_VERSION);
  WS_DEBUG_PRINTLN("API: Version 2");
  WS_DEBUG_PRINT("Board ID: ");
  WS_DEBUG_PRINTLN(BOARD_ID);
  WS_DEBUG_PRINT("Adafruit.io User: ");
  WS_DEBUG_PRINTLN(WsV2._configV2.aio_user);
  WS_DEBUG_PRINT("WiFi Network: ");
  WS_DEBUG_PRINTLN(WsV2._configV2.network.ssid);

  char sMAC[18] = {0};
  sprintf(sMAC, "%02X:%02X:%02X:%02X:%02X:%02X", WsV2._macAddrV2[0],
          WsV2._macAddrV2[1], WsV2._macAddrV2[2], WsV2._macAddrV2[3],
          WsV2._macAddrV2[4], WsV2._macAddrV2[5]);
  WS_DEBUG_PRINT("MAC Address: ");
  WS_DEBUG_PRINTLN(sMAC);
  WS_DEBUG_PRINTLN("-------------------------------");

// (ESP32-Only) Print reason why device was reset
#ifdef ARDUINO_ARCH_ESP32
  esp_reset_reason_t r = esp_reset_reason();
  WS_DEBUG_PRINT("ESP Reset Reason: ");
  WS_DEBUG_PRINTLN(resetReasonName(r));
#endif
}

/**************************************************************************/
/*!
    @brief    Connects to Adafruit IO+ Wippersnapper_V2 broker.
*/
/**************************************************************************/
void Wippersnapper_V2::connectV2() {
  WS_DEBUG_PRINTLN("Adafruit.io WipperSnapper");

  // Dump device info to the serial monitor
  printDeviceInfoV2();

  // enable global WDT
  WsV2.enableWDTV2(WS_WDT_TIMEOUT);

  // Generate device identifier
  WS_DEBUG_PRINTLN("Generating device UID...");
  if (!generateDeviceUIDV2()) {
    haltErrorV2("Unable to generate Device UID");
  }
  WS_DEBUG_PRINTLN("Device UID generated successfully!");

  // Configures an Adafruit Arduino MQTT object
  WS_DEBUG_PRINTLN("Setting up MQTT client...");
  setupMQTTClientV2(_device_uidV2);
  WS_DEBUG_PRINTLN("Set up MQTT client successfully!");

  WS_DEBUG_PRINTLN("Generating device's MQTT topics...");
  if (!generateWSTopicsV2()) {
    haltErrorV2("Unable to allocate space for MQTT topics");
  }
  WS_DEBUG_PRINTLN("Generated device's MQTT topics successfully!");

  // Connect to Network
  WS_DEBUG_PRINTLN("Running Network FSM...");
  // Run the network fsm
  runNetFSMV2();
  WsV2.feedWDTV2();

#ifdef USE_DISPLAY
  WsV2._ui_helper->set_load_bar_icon_complete(loadBarIconCloud);
  WsV2._ui_helper->set_label_status("Sending device info...");
#endif

  WS_DEBUG_PRINTLN("Performing checkin handshake...");
  // Publish the checkin request
  if (!CreateCheckinRequest()) {
    haltErrorV2("Unable to publish checkin request");
  }
  // Handle the checkin response
  HandleCheckinResponse();

  // Set the status LED to green to indicate successful configuration
  // TODO: Use the brightness value from the .json file
  setStatusLEDColor(0x00A300, 155);
  delay(20);
  // Set the status LED to off during app runtime
  setStatusLEDColor(0x000000, 255);

// switch to monitor screen
#ifdef USE_DISPLAY
  WS_DEBUG_PRINTLN("Clearing loading screen...");
  WsV2._ui_helper->clear_scr_load();
  WS_DEBUG_PRINTLN("building monitor screen...");
  WsV2._ui_helper->build_scr_monitor();
#endif
}

/**************************************************************************/
/*!
    @brief    Processes incoming commands and handles network connection.
    @returns  Network status, as ws_status_t.
*/
/**************************************************************************/
ws_status_t Wippersnapper_V2::runV2() {
  // Check networking
  runNetFSMV2();
  WsV2.feedWDTV2();
  pingBrokerV2();

  // Process all incoming packets from Wippersnapper_V2 MQTT Broker
  WsV2._mqttV2->processPackets(10);
  WsV2.feedWDTV2();

  // Process digital inputs, digitalGPIO module
  WsV2._digitalGPIOV2->processDigitalInputs();
  WsV2.feedWDTV2();

  // Process analog inputs
  WsV2._analogIOV2->update();
  WsV2.feedWDTV2();

  // Process I2C sensor events
  if (WsV2._isI2CPort0InitV2)
    WsV2._i2cPort0V2->update();
  WsV2.feedWDTV2();

  // Process DS18x20 sensor events
  WsV2._ds18x20ComponentV2->update();
  WsV2.feedWDTV2();

  // Process UART sensor events
  WsV2._uartComponentV2->update();
  WsV2.feedWDTV2();

  return WS_NET_CONNECTED; // TODO: Make this funcn void!
}
