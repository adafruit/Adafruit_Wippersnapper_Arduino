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
 * Copyright (c) Brent Rubell 2020-2025 for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper_V2.h"

Wippersnapper_V2 WsV2;

/*!
    @brief    Wippersnapper_V2 constructor
*/
Wippersnapper_V2::Wippersnapper_V2()
    : _mqttV2(nullptr), sensor_model(nullptr), error_controller(nullptr),
      digital_io_controller(nullptr), analogio_controller(nullptr),
      _ds18x20_controller(nullptr), _gps_controller(nullptr),
      _i2c_controller(nullptr), _uart_controller(nullptr),
      _pixels_controller(nullptr), _pwm_controller(nullptr),
      _servo_controller(nullptr) {
  // Initialize model classes
  sensor_model = new SensorModel();

  // Initialize controller classes
  digital_io_controller = new DigitalIOController();
  analogio_controller = new AnalogIOController();
  _ds18x20_controller = new DS18X20Controller();
  _gps_controller = new GPSController();
  _i2c_controller = new I2cController();
  _uart_controller = new UARTController();
  _pixels_controller = new PixelsController();
  _pwm_controller = new PWMController();
  _servo_controller = new ServoController();
#ifdef ARDUINO_ARCH_ESP32
  _sleep_controller = new SleepController();
#endif
}

/*!
    @brief    Wippersnapper_V2 destructor 
*/
Wippersnapper_V2::~Wippersnapper_V2() {
  disconnect();
  delete this->sensor_model;
  // delete this->error_controller;
  delete this->digital_io_controller;
  delete this->analogio_controller;
  delete this->_ds18x20_controller;
  delete this->_gps_controller;
  delete this->_i2c_controller;
  delete this->_uart_controller;
  delete this->_pixels_controller;
  delete this->_pwm_controller;
  delete this->_servo_controller;
#ifdef ARDUINO_ARCH_ESP32
  delete this->_sleep_controller;
#endif
}

/*!
    @brief    Disconnects from Adafruit IO+ Wippersnapper_V2.
*/
void Wippersnapper_V2::disconnect() { _disconnect(); }

// Concrete class definition for abstract classes

/*!
    @brief    Connects to wireless network.
*/
void Wippersnapper_V2::_connect() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::_connect()");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
    @brief    Disconnect Wippersnapper MQTT session and network.
*/
void Wippersnapper_V2::_disconnect() {
  WS_DEBUG_PRINTLN("WIppersnapper_V2::_disconnect");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
    @brief    Sets the network interface's unique identifer, typically the
              MAC address.
*/
void Wippersnapper_V2::getMacAddr() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::getMacAddr");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
    @brief    Gets the network's RSSI.
    @return   int32_t RSSI value, 0 to 255, in dB
*/
int32_t Wippersnapper_V2::getRSSI() {
  WS_DEBUG_PRINTLN("Wiippersnapper_V2::getRSSI");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return 0;
}

/*!
    @brief    Sets up the MQTT client session.
    @param    clientID
              A unique client identifier string.
*/
void Wippersnapper_V2::setupMQTTClient(const char * /*clientID*/) {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::setupMQTTClient");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
    @brief    Returns the network's connection status
    @returns  Network status as ws_status_t.
*/
ws_status_t Wippersnapper_V2::networkStatus() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::networkStatus");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return WS_IDLE;
}

/*!
    @brief    Sets the device's wireless network credentials.
    @param    ssid
              Your wireless network's SSID
    @param    ssidPassword
              Your wireless network's password.
*/
void Wippersnapper_V2::set_ssid_pass(const char * /*ssid*/,
                                     const char * /*ssidPassword*/) {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::set_ssid_pass");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
    @brief    Sets the device's wireless network credentials from the
              secrets.json configuration file.
*/
void Wippersnapper_V2::set_ssid_pass() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::set_ssid_pass");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
@brief   Performs a scan of local WiFi networks.
@returns True if `_network_ssid` is found, False otherwise.
*/
bool Wippersnapper_V2::check_valid_ssid() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::check_valid_ssid");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
  return false;
}

/*!
    @brief    Configures the device's Adafruit IO credentials. This method
              should be used only if filesystem-backed provisioning is
              not avaliable.
*/
void Wippersnapper_V2::set_user_key() {
  WS_DEBUG_PRINTLN("Wippersnapper_V2::set_user_key");
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/*!
    @brief    Provisions a WipperSnapper device with its network
              configuration and Adafruit IO credentials.
*/
void Wippersnapper_V2::provision() {
  // Obtain device's MAC address
  getMacAddr();

  // Initialize the status LED for signaling FS errors
  initStatusLED();

// Initialize the filesystem
#ifdef USE_TINYUSB
  _fileSystemV2 = new Wippersnapper_FS();
#elif defined(USE_LITTLEFS)
  _littleFSV2 = new WipperSnapper_LittleFS();
#endif

// Determine if app is in SDLogger mode
#ifdef USE_TINYUSB
  _fileSystemV2->GetSDCSPin();
#elif defined(USE_LITTLEFS)
  _littleFSV2->GetSDCSPin();
#elif defined(OFFLINE_MODE_WOKWI)
  WS_DEBUG_PRINTLN("Wokwi offline mode detected, setting SD CS pin to 15");
  WsV2.pin_sd_cs = 15;
#endif
  WS_DEBUG_PRINT("SD CS Pin: ");
  WS_DEBUG_PRINTLN(WsV2.pin_sd_cs);
  WsV2._sdCardV2 = new ws_sdcard();
  WS_DEBUG_PRINTLN("Is SD Card initialized?");
  WS_DEBUG_PRINTLN(WsV2._sdCardV2->isSDCardInitialized());
  if (WsV2._sdCardV2->isSDCardInitialized()) {
    return; // SD card initialized, cede control back to loop()
  } else {
#ifdef BUILD_OFFLINE_ONLY
    haltErrorV2("SD initialization failed.\nDo not reformat the card!\nIs the "
                "card correctly inserted?\nIs there a wiring/soldering "
                "problem\nIs the config.json file malformed?");
#endif
    // SD card not initialized, so just continue with online-mode provisioning
  }

#ifdef USE_TINYUSB
  _fileSystemV2->parseSecrets();
#elif defined(USE_LITTLEFS)
  _littleFSV2->parseSecrets();
#else
  check_valid_ssid(); // non-fs-backed, sets global credentials within network
                      // iface
#endif
  // Set the status pixel's brightness
  setStatusLEDBrightness(WsV2._configV2.status_pixel_brightness);
  // Set device's wireless credentials
  set_ssid_pass();
}

/*!
    @brief    Handles a Checkin Response message and initializes the
              device's GPIO classes.
    @param    stream
              Incoming data stream from buffer.
    @returns  True if Checkin Response decoded and parsed successfully,
              False otherwise.
*/
bool handleCheckinResponse(pb_istream_t *stream) {
  // Decode the Checkin Response message
  WS_DEBUG_PRINTLN("[app] Decoding Checkin Response message");
  if (!WsV2.CheckInModel->ProcessResponse(stream)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode Checkin Response message");
    return false;
  }
  WsV2.runNetFSMV2();

  // Configure controller settings using Response
  WS_DEBUG_PRINTLN("[app] Configuring controllers");
  WsV2.CheckInModel->ConfigureControllers();
  WsV2.runNetFSMV2();

  // Publish the complete response message to indicate the checkin
  // routine is done and the device is ready for use
  WS_DEBUG_PRINTLN("[app] Publishing complete response message");
  return WsV2.CheckInModel->Complete();
}

// Decoders //

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
bool routeBrokerToDevice(pb_istream_t *stream, const pb_field_t *field,
                         void **arg) {
  (void)arg;

  if (stream == nullptr || field == nullptr) {
    WS_DEBUG_PRINTLN("ERROR: Null stream or field in routeBrokerToDevice");
    return false;
  }

  // Pass to class' router based on tag type
  WS_DEBUG_PRINT("Handling BrokerToDevice message with tag: ");
  switch (field->tag) {
  case ws_signal_BrokerToDevice_error_tag:
    WS_DEBUG_PRINTLN("error");
    return WsV2.error_controller->Router(stream);
  case ws_signal_BrokerToDevice_checkin_tag:
    WS_DEBUG_PRINTLN("checkin");
    return handleCheckinResponse(stream);
  case ws_signal_BrokerToDevice_digitalio_tag:
    return WsV2.digital_io_controller->Router(stream);
  case ws_signal_BrokerToDevice_analogio_tag:
    return WsV2.analogio_controller->Router(stream);
  case ws_signal_BrokerToDevice_pixels_tag:
    return WsV2._pixels_controller->Router(stream);
  case ws_signal_BrokerToDevice_pwm_tag:
    return WsV2._pwm_controller->Router(stream);
  case ws_signal_BrokerToDevice_servo_tag:
    return WsV2._servo_controller->Router(stream);
  case ws_signal_BrokerToDevice_ds18x20_tag:
    return WsV2._ds18x20_controller->Router(stream);
  case ws_signal_BrokerToDevice_i2c_tag:
    return WsV2._i2c_controller->Router(stream);
  case ws_signal_BrokerToDevice_uart_tag:
    return WsV2._uart_controller->Router(stream);
  default:
    WS_DEBUG_PRINTLN("WARNING: Unhandled BrokerToDevice message tag!");
    return false;
  }
}

/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /ws-b2d/ "signal topic".
    @param    data
                Data (payload) from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
void cbBrokerToDevice(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("=> New B2D message!");
  ws_signal_BrokerToDevice msg_signal = ws_signal_BrokerToDevice_init_default;

  // Configure the payload callback
  msg_signal.cb_payload.funcs.decode = routeBrokerToDevice;

  // Decode message
  pb_istream_t istream = pb_istream_from_buffer((uint8_t *)data, len);
  if (!pb_decode(&istream, ws_signal_BrokerToDevice_fields, &msg_signal)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode BrokerToDevice message!");
    return;
  }

  WS_DEBUG_PRINTLN("=> B2D message decoded successfully!");
}

/*!
    @brief    Decodes and parses a buffer containing configuration
              messages from the SD card.
*/
void callDecodeB2D() {
  for (size_t i = 0; i < WsV2._sharedConfigBuffers.size(); i++) {
    ws_signal_BrokerToDevice msg_signal = ws_signal_BrokerToDevice_init_default;
    // Configure the payload callback
    msg_signal.cb_payload.funcs.decode = routeBrokerToDevice;
    const std::vector<uint8_t> &buffer = WsV2._sharedConfigBuffers[i];
    pb_istream_t istream = pb_istream_from_buffer(buffer.data(), buffer.size());
    // Decode the message
    if (!pb_decode(&istream, ws_signal_BrokerToDevice_fields, &msg_signal)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to decode BrokerToDevice message!");
      continue; // Skip this message and move on!
    }
  }
}

/*!
    @brief    Attempts to generate unique device identifier.
    @returns  True if device identifier generated successfully,
              False otherwise.
*/
bool Wippersnapper_V2::generateDeviceUID() {
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

/*!
    @brief    Generates device-specific Wippersnapper control topics and
              subscribes to them.
    @returns  True if memory for control topics allocated successfully,
                False otherwise.
*/
bool Wippersnapper_V2::generateWSTopics() {
  // Calculate length for topic strings
  size_t lenSignalTopic = strlen(WsV2._configV2.aio_user) +
                          WS_TOPIC_PREFIX_LEN + strlen(_device_uidV2) + 1;

  // Attempt to allocate memory for the broker-to-device topic
#ifdef USE_PSRAM
  WsV2._topicB2d = (char *)ps_malloc(sizeof(char) * lenSignalTopic);
#else
  WsV2._topicB2d = (char *)malloc(sizeof(char) * lenSignalTopic);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicB2d == NULL)
    return false;
  // Build the broker-to-device topic
  snprintf(WsV2._topicB2d, lenSignalTopic, "%s/ws-b2d/%s",
           WsV2._configV2.aio_user, _device_uidV2);
  // Subscribe to broker-to-device topic
  _subscribeB2d = new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicB2d, 1);
  WsV2._mqttV2->subscribe(_subscribeB2d);
  _subscribeB2d->setCallback(cbBrokerToDevice);

  // Create global device to broker topic
  // Attempt to allocate memory for the broker-to-device topic
#ifdef USE_PSRAM
  WsV2._topicD2b = (char *)ps_malloc(sizeof(char) * lenSignalTopic);
#else
  WsV2._topicD2b = (char *)malloc(sizeof(char) * lenSignalTopic);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicD2b == NULL) {
    // Release resources and return false
    free(WsV2._topicB2d);
    WsV2._topicB2d = NULL;
    delete _subscribeB2d;
    _subscribeB2d = NULL;
    return false;
  }

  // Build the broker-to-device topic
  snprintf(WsV2._topicD2b, lenSignalTopic, "%s/ws-d2b/%s",
           WsV2._configV2.aio_user, _device_uidV2);
  return true;
}

/*!
    @brief    Writes an error message to the serial and the filesystem,
                blinks WS_LED_STATUS_ERROR_RUNTIME pattern and hangs.
    @param    error
              The error message to write to the serial and filesystem.
*/
void Wippersnapper_V2::errorWriteHangV2(const char *error) {
  // Print error
  WS_DEBUG_PRINTLN(error);
#ifdef USE_TINYUSB
  _fileSystemV2->writeToBootOut(error);
  TinyUSBDevice.attach();
  delay(500);
#endif
  // Signal and hang forever
  while (1) {
    WS_DEBUG_PRINTLN("ERROR: Halted execution");
    WS_DEBUG_PRINTLN(error);
    WsV2.feedWDTV2();
    statusLEDBlink(WS_LED_STATUS_ERROR_RUNTIME);
    delay(1000);
  }
}

/*!
    @brief    Checks network and MQTT connectivity. Handles network
              re-connection and mqtt re-establishment.
*/
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
      if (networkStatus() == WS_NET_CONNECTED) {
        WS_DEBUG_PRINTLN("Connected to WiFi!");
        fsmNetwork = FSM_NET_ESTABLISH_MQTT;
        break;
      }
      fsmNetwork = FSM_NET_ESTABLISH_NETWORK;
      break;
    case FSM_NET_ESTABLISH_NETWORK:
      WS_DEBUG_PRINTLN("Establishing network connection...");
      WS_PRINTER.flush();
      // Perform a WiFi scan and check if SSID within
      // secrets.json is within the scanned SSIDs
      WS_DEBUG_PRINT("Performing a WiFi scan for SSID...");
      if (!check_valid_ssid()) {
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
        _connect();
        feedWDTV2();
        // did we connect?
        if (networkStatus() == WS_NET_CONNECTED)
          break;
        maxAttempts--;
      }
      // Validate connection
      if (networkStatus() != WS_NET_CONNECTED) {
        WS_DEBUG_PRINTLN("ERROR: Unable to connect to WiFi!");
        haltErrorV2("ERROR: Unable to connect to WiFi, rebooting soon...",
                    WS_LED_STATUS_WIFI_CONNECTING);
      }

      fsmNetwork = FSM_NET_CHECK_NETWORK;
      break;
    case FSM_NET_ESTABLISH_MQTT:
      WsV2._mqttV2->setKeepAliveInterval(WS_KEEPALIVE_INTERVAL_MS / 1000);
      // Attempt to connect
      maxAttempts = 5;
      while (maxAttempts > 0) {
        WS_DEBUG_PRINT("Connecting to AIO MQTT (attempt #");
        WS_DEBUG_PRINT(5 - maxAttempts);
        WS_DEBUG_PRINTLN(")");
        WS_PRINTER.flush();
        WS_DEBUG_PRINT("WiFi Status: ");
        WS_DEBUG_PRINTLN(networkStatus());
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

/*!
    @brief    Prints an error to the serial and halts the hardware until
              the WDT bites.
    @param    error
              The desired error to print to serial.
    @param    ledStatusColor
              The desired color to blink.
    @param    reboot
              If true, the device will reboot after the WDT bites.
              If false, the device will not allow the WDT to bite and
              instead hang indefinitely, holding the WIPPER drive open
*/
void Wippersnapper_V2::haltErrorV2(const char *error,
                                   ws_led_status_t ledStatusColor,
                                   bool reboot) {
  WS_DEBUG_PRINT("ERROR ");
  if (reboot) {
    WS_DEBUG_PRINT("[RESET]: ");
  } else {
    WS_DEBUG_PRINT("[HANG]: ");
  }
  WS_DEBUG_PRINTLN(error);
  statusLEDSolid(ledStatusColor);
  for (;;) {
    if (!reboot) {
      WsV2.feedWDTV2(); // Feed the WDT indefinitely to hold the WIPPER drive
                        // open
    } else {
// Let the WDT fail out and reset!
#ifndef ARDUINO_ARCH_ESP8266
      delay(1000);
#else
      // Calls to delay() and yield() feed the ESP8266's
      // hardware and software watchdog timers, delayMicroseconds does not.
      delayMicroseconds(1000000);
#endif
    }
  }
}

/*!
    @brief    Wraps and publishes a DeviceToBroker signal message
                to Adafruit IO.
    @param    which_payload
              The DeviceToBroker payload type.
    @param    payload
              The payload to publish.
    @returns  True if the signal message published successfully,
              False otherwise.
*/
bool Wippersnapper_V2::PublishD2b(pb_size_t which_payload, void *payload) {
  WS_DEBUG_PRINTLN("=> Publishing DeviceToBroker signal message...");

  // Alloc memory on heap for the DeviceToBroker message
  ws_signal_DeviceToBroker *msg =
      (ws_signal_DeviceToBroker *)malloc(sizeof(ws_signal_DeviceToBroker));
  if (msg == NULL) {
    WS_DEBUG_PRINTLN("ERROR: Failed to allocate memory for D2B message!");
    return false;
  }

  // Initialize DeviceToBroker message
  memset(msg, 0, sizeof(ws_signal_DeviceToBroker));
  *msg = ws_signal_DeviceToBroker_init_zero;

  // Fill generic signal payload with the payload from the args.
  switch (which_payload) {
  case ws_signal_DeviceToBroker_error_tag:
    WS_DEBUG_PRINTLN("Signal type: Error");
    msg->which_payload = ws_signal_DeviceToBroker_error_tag;
    msg->payload.error = *(ws_error_ErrorD2B *)payload;
    break;
  case ws_signal_DeviceToBroker_checkin_tag:
    WS_DEBUG_PRINTLN("Signal type: Checkin");
    msg->which_payload = ws_signal_DeviceToBroker_checkin_tag;
    msg->payload.checkin = *(ws_checkin_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_digitalio_tag:
    WS_DEBUG_PRINTLN("Signal type: DigitalIO");
    msg->which_payload = ws_signal_DeviceToBroker_digitalio_tag;
    msg->payload.digitalio = *(ws_digitalio_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_analogio_tag:
    WS_DEBUG_PRINTLN("Signal type: AnalogIO");
    msg->which_payload = ws_signal_DeviceToBroker_analogio_tag;
    msg->payload.analogio = *(ws_analogio_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_servo_tag:
    WS_DEBUG_PRINTLN("Signal type: Servo");
    msg->which_payload = ws_signal_DeviceToBroker_servo_tag;
    msg->payload.servo = *(ws_servo_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_pwm_tag:
    WS_DEBUG_PRINTLN("Signal type: PWM");
    msg->which_payload = ws_signal_DeviceToBroker_pwm_tag;
    msg->payload.pwm = *(ws_pwm_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_pixels_tag:
    WS_DEBUG_PRINTLN("Signal type: Pixels");
    msg->which_payload = ws_signal_DeviceToBroker_pixels_tag;
    msg->payload.pixels = *(ws_pixels_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_ds18x20_tag:
    WS_DEBUG_PRINTLN("Signal type: DS18x20");
    msg->which_payload = ws_signal_DeviceToBroker_ds18x20_tag;
    msg->payload.ds18x20 = *(ws_ds18x20_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_uart_tag:
    WS_DEBUG_PRINTLN("Signal type: UART");
    msg->which_payload = ws_signal_DeviceToBroker_uart_tag;
    msg->payload.uart = *(ws_uart_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_i2c_tag:
    WS_DEBUG_PRINTLN("Signal type: I2C");
    msg->which_payload = ws_signal_DeviceToBroker_i2c_tag;
    msg->payload.i2c = *(ws_i2c_D2B *)payload;
    break;
  case ws_signal_DeviceToBroker_gps_tag:
    WS_DEBUG_PRINTLN("Signal type: GPS");
    msg->which_payload = ws_signal_DeviceToBroker_gps_tag;
    msg->payload.gps = *(ws_gps_D2B *)payload;
    break;
  default:
    WS_DEBUG_PRINTLN("ERROR: Invalid signal payload type, bailing out!");
    free(msg);
    return false;
  }

  // Get the encoded size of the signal message
  size_t szMessageBuf;
  if (!pb_get_encoded_size(&szMessageBuf, ws_signal_DeviceToBroker_fields,
                           msg)) {
    WS_DEBUG_PRINTLN(
        "ERROR: Unable to get encoded size of signal message, bailing out!");
    free(msg);
    return false;
  }

  // Size the message buffer to fit the encoded d2b message
  uint8_t msgBuf[szMessageBuf];
  // Encode the signal message
  WS_DEBUG_PRINT("Encoding d2b message...");
  pb_ostream_t stream = pb_ostream_from_buffer(msgBuf, szMessageBuf);
  if (!ws_pb_encode(&stream, ws_signal_DeviceToBroker_fields, msg)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode d2b message, bailing out!");
    free(msg);
    return false;
  }
  WS_DEBUG_PRINTLN("Encoded!");

  // Check that we are still connected
  runNetFSMV2();
  WsV2.feedWDTV2();

  // Attempt to publish the signal message to the broker
  WS_DEBUG_PRINT("Publishing signal message to broker...");
  if (!WsV2._mqttV2->publish(WsV2._topicD2b, msgBuf, szMessageBuf, 1)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to publish d2b message to broker!");
    free(msg);
    return false;
  }

  WS_DEBUG_PRINTLN("Published!");
  // Free the allocated message's memory
  free(msg);
  return true;
}

/*!
    @brief  Pings the MQTT broker within the keepalive interval
            to keep the connection alive. Blinks the keepalive LED
            every STATUS_LED_KAT_BLINK_TIME milliseconds.
*/
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
    WS_DEBUG_PRINTLN(getRSSI());
  }
  // Blink the status LED to indicate that the device is still alive
  BlinkKATStatus();
}

/*!
    @brief  Blinks the status LED every STATUS_LED_KAT_BLINK_TIME
            milliseconds to indicate that the device is still alive.
*/
void Wippersnapper_V2::BlinkKATStatus() {
  if (millis() > (_prvKATBlinkV2 + STATUS_LED_KAT_BLINK_TIME)) {
    statusLEDBlink(WS_LED_STATUS_KAT);
    _prvKATBlinkV2 = millis();
  }
}

/*!
    @brief    Feeds the WDT to prevent hardware reset.
*/
void Wippersnapper_V2::feedWDTV2() {
#ifndef OFFLINE_MODE_WOKWI
  // TODO: This is a temporary fix for watchdog.reset() not firing
  Watchdog.reset();
// esp_task_wdt_reset(); // TODO: Putback for ESP32 WDT
#endif
}

/*!
    @brief  Enables the watchdog timer.
    @param  timeoutMS
            The desired amount of time to elapse before
            the WDT executes.
*/
void Wippersnapper_V2::enableWDTV2(int timeoutMS) {
#ifndef ARDUINO_ARCH_RP2040
  Watchdog.disable();
#endif
  if (Watchdog.enable(timeoutMS) == 0) {
    WsV2.haltErrorV2("WDT initialization failure!");
  }
}

/*!
    @brief  Process all incoming packets from the
            Adafruit IO MQTT broker. Handles network
            connectivity.
*/
void Wippersnapper_V2::processPacketsV2() {
  // runNetFSMV2(); // NOTE: Removed for now, causes error with virtual
  // _connect() method when caused with WsV2 object in another file.
  WsV2.feedWDTV2();
  // Process all incoming packets from Wippersnapper_V2 MQTT Broker
  WsV2._mqttV2->processPackets(10);
}

/*!
    @brief    Prints information about the WsV2 device to the serial monitor.
*/
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

/*!
    @brief    Connects to Adafruit IO+ Wippersnapper_V2 broker.
*/
void Wippersnapper_V2::connect() {
  WS_DEBUG_PRINTLN("Adafruit.io WipperSnapper");
  // Dump device info to the serial monitor
  printDeviceInfoV2();

  // TODO: Does this need to be here?
  WsV2.error_controller = new ErrorController();

  // enable global WDT
  WsV2.enableWDTV2(WS_WDT_TIMEOUT);

  // Generate device identifier
  if (!generateDeviceUID()) {
    haltErrorV2("Unable to generate Device UID");
  }

  // If we are running in offline mode, we skip the network setup
  // and MQTT connection process and jump to the offline device config process
  // NOTE: After this, bail out of this function and run the app loop!!!
  if (WsV2._sdCardV2->isModeOffline() == true) {
    WS_DEBUG_PRINTLN("[Offline] Running device configuration...");
// If debug mode, wait for serial config
#ifdef OFFLINE_MODE_DEBUG
    WsV2._sdCardV2->waitForSerialConfig();
#endif
    // Parse the JSON file
    if (!WsV2._sdCardV2->parseConfigFile())
      haltErrorV2("Failed to parse config.json!");
    WS_DEBUG_PRINTLN("[Offline] Attempting to configure hardware...");
#ifndef OFFLINE_MODE_DEBUG
    if (!WsV2._sdCardV2->CreateNewLogFile())
      haltErrorV2("Unable to create new .log file on SD card!");
#endif
    // Call the TL signal decoder to parse the incoming JSON data
    callDecodeB2D();
    WS_DEBUG_PRINTLN("[Offline] Hardware configured, skipping network setup "
                     "and running app...");
    // Blink status LED to green to indicate successful configuration
    setStatusLEDColor(0x00A300, WsV2.status_pixel_brightnessV2 * 255.0);
    delay(500);
    setStatusLEDColor(0x000000, WsV2.status_pixel_brightnessV2 * 255.0);
    return;
  } else {
    WS_DEBUG_PRINTLN("Running in online mode...");
  }

  // Configures an Adafruit Arduino MQTT object
  WS_DEBUG_PRINTLN("Setting up MQTT client...");
  setupMQTTClient(_device_uidV2);
  WS_DEBUG_PRINTLN("Set up MQTT client successfully!");

  WS_DEBUG_PRINTLN("Generating device's MQTT topics...");
  if (!generateWSTopics()) {
    haltErrorV2("Unable to allocate space for MQTT topics");
  }
  WS_DEBUG_PRINTLN("Generated device's MQTT topics successfully!");

  // Connect to Network
  WS_DEBUG_PRINTLN("Running Network FSM...");
  // Run the network fsm
  runNetFSMV2();
  WsV2.feedWDTV2();

  // TODO: Possibly refactor checkin process into its own function
  // or component class for clarity
  // TODO: Remove logging from checkin process,
  // but only after we test on staging
  WsV2.CheckInModel = new CheckinModel();
  WS_DEBUG_PRINTLN("Creating checkin request...");
  // Publish the checkin request
  if (!WsV2.CheckInModel->Checkin(BOARD_ID, WS_VERSION)) {
    haltErrorV2("ERROR: Unable to create and/or checkin request");
  }
  WS_DEBUG_PRINTLN("Published checkin request...");
  // Poll for checkin response
  WS_DEBUG_PRINTLN("Waiting for checkin response...");
  WsV2.feedWDTV2();
  // NOTE: If we do not receive a response within a certain time frame,
  // the WDT will reset the device and try again
  while (!WsV2.CheckInModel->GotResponse()) {
    WsV2._mqttV2->processPackets(10); // TODO: Test with lower timeout value
    pingBrokerV2();                   // Keep MQTT connection alive
  }
  WS_DEBUG_PRINTLN("Completed checkin process!");
  // Perform cleanup for checkin process, we don't need it anymore
  WS_DEBUG_PRINT("Free heap before freeing checkin model: ");
  WS_DEBUG_PRINTLN(ESP.getFreeHeap());
  delete WsV2.CheckInModel;
  WsV2.CheckInModel = nullptr;
  WS_DEBUG_PRINT("Free heap going into loop: ");
  WS_DEBUG_PRINTLN(ESP.getFreeHeap());

  // Set the status LED to green to indicate successful configuration
  setStatusLEDColor(0x00A300, WsV2.status_pixel_brightnessV2);
  delay(100);
  // Set the status LED to off during app runtime
  setStatusLEDColor(0x000000, WsV2.status_pixel_brightnessV2);

  WS_DEBUG_PRINTLN("Running app loop...");
}

/*!
    @brief    Determines which loop() to call depending on the power mode.
*/
void Wippersnapper_V2::run() {
#ifdef ARDUINO_ARCH_ESP32
    if (_sleep_controller->IsSleepLoop()) {
        loopSleep();
    } else {
        loop();
    }
#else
    loop();
#endif
}

void Wippersnapper_V2::loop() {
      WsV2.feedWDTV2();
  if (!WsV2._sdCardV2->isModeOffline()) {
    // Handle networking functions
    runNetFSMV2();
    pingBrokerV2();
    // Process all incoming packets from Wippersnapper_V2 MQTT Broker
    WsV2._mqttV2->processPackets(10);
  } else {
    BlinkKATStatus(); // Offline Mode - Blink every KAT interval
  }

  // Process all digital events
  WsV2.digital_io_controller->update();

  // Process all analog inputs
  WsV2.analogio_controller->update();

  // Process all DS18x20 sensor events
  WsV2._ds18x20_controller->update();

  // Process I2C driver events
  WsV2._i2c_controller->update();

  // Process UART driver events
  WsV2._uart_controller->update();

  // Process GPS controller events
  WsV2._gps_controller->update();
}

#ifdef ARDUINO_ARCH_ESP32
/*!
    @brief    loop() variant that uses a component-driven readiness tracking and a global timer for run duration and entrypoints for sleep management.
*/
void Wippersnapper_V2::loopSleep() {
    // TODO: loop() but with a global timer for run duration, and checks
}
#endif