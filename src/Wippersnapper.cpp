/*!
 * @file Wippersnapper.cpp
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
 * Copyright (c) Brent Rubell 2020-2021 for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper.h"

Wippersnapper WS;

Wippersnapper::Wippersnapper() {
  _mqtt = 0; // MQTT Client object

  // IO creds
  _username = 0;
  _key = 0;

  // Reserved MQTT Topics
  _topic_description = 0;
  _topic_description_status = 0;
  _topic_signal_device = 0;
  _topic_signal_brkr = 0;
  _err_topic = 0;
  _throttle_topic = 0;
  _err_sub = 0;
  _throttle_sub = 0;
};

/**************************************************************************/
/*!
    @brief    Wippersnapper destructor
*/
/**************************************************************************/
Wippersnapper::~Wippersnapper() {
  // free topics
  free(_topic_description);
  free(_topic_signal_device);
  free(_topic_signal_brkr);
  free(_err_sub);
  free(_throttle_sub);
}

/****************************************************************************/
/*!
    @brief    Initializes provisioning, either the native USB FS or
              NVS (ESP32)
*/
/****************************************************************************/
void Wippersnapper::startProvisioning() {
#ifdef USE_TINYUSB
  // Filesystem-based provisioning flow
  _fileSystem = new Wippersnapper_FS(); // Initialize the QSPI flash FS
#elif defined(USE_NVS)
  _nvs = new Wippersnapper_ESP32_nvs();
#endif
}

/****************************************************************************/
/*!
    @brief    Validates if file system contains secret credentials for
              Adafruit IO and wireless network.
*/
/****************************************************************************/
void Wippersnapper::validateProvisioningSecrets() {
#ifdef USE_TINYUSB
  // check for secrets.json, create if doesn't exist
  if (!_fileSystem->configFileExists()) {
    // create config file on filesystem
    _fileSystem->createConfigFileSkel();
  }
#elif defined(USE_NVS)
  if (!_nvs->validateNVSConfig()) {
    WS_DEBUG_PRINTLN(
        "ERROR: NVS partition or credentials not found - was NVS flashed?");
    while (1)
      yield();
  }
#endif
}

/****************************************************************************/
/*!
    @brief    Parses and stores data from the secrets.json file or a captive
                provisioning portal.
    @returns  True if secrets are parsed and set stored successfully,
                False otherwise.
*/
/****************************************************************************/
bool Wippersnapper::parseProvisioningSecrets() {
  bool is_successful = false;
#ifdef USE_TINYUSB
  is_successful = _fileSystem->parseSecrets();
#elif defined(USE_NVS)
  is_successful = _nvs->setNVSConfig();
#endif
  return is_successful;
}

/****************************************************************************/
/*!
    @brief    Configures the device's Adafruit IO credentials. This method
              should be used only if provisioning is not avaliable.
    @param    aio_username
              Your Adafruit IO username.
    @param    aio_key
              Your Adafruit IO active key.
*/
/****************************************************************************/
void Wippersnapper::set_user_key(const char *aio_username,
                                 const char *aio_key) {
  WS._username = aio_username;
  WS._key = aio_key;
}

/****************************************************************************/
/*!
    @brief    Configures the device's Adafruit IO credentials from the
                secrets.json file.
*/
/****************************************************************************/
void Wippersnapper::set_user_key() {
#ifdef USE_TINYUSB
  if (_fileSystem->io_username != NULL) {
    WS._username = _fileSystem->io_username;
  } else {
    WS_DEBUG_PRINTLN(
        "ERROR: Adafruit IO username not set correctly in secrets.json.");
    while (1)
      yield();
  }

  if (_fileSystem->io_key != NULL) {
    WS._key = _fileSystem->io_key;
  } else {
    WS_DEBUG_PRINTLN(
        "ERROR: Adafruit IO key not set correctly in secrets.json.");
    while (1)
      yield();
  }
#endif
}

// Decoders //
/****************************************************************************/
/*!
    @brief    Configures a pin according to a
                wippersnapper_pin_v1_ConfigurePinRequest message.
    @param    pinMsg
              Pointer to a wippersnapper_pin_v1_ConfigurePinRequest message.
    @returns  True if pin configured successfully, False otherwise.
*/
/****************************************************************************/
bool Wippersnapper::configurePinRequest(
    wippersnapper_pin_v1_ConfigurePinRequest *pinMsg) {
  WS_DEBUG_PRINTLN("configurePinRequest");

  bool is_success = true;
  char *pinName = pinMsg->pin_name + 1;
  int pin = atoi(pinName);

  // Decode pin mode
  if (pinMsg->mode == wippersnapper_pin_v1_Mode_MODE_DIGITAL) {
    if (pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
      // Initialize GPIO pin
      WS._digitalGPIO->initDigitalPin(pinMsg->direction, pin, pinMsg->period);
    } else if (
        pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
      // Delete digital GPIO pin
      WS._digitalGPIO->deinitDigitalPin(pinMsg->direction, pin);
    } else {
      WS_DEBUG_PRINTLN("ERROR: Could not decode digital pin request type");
    }
  }

  else if (pinMsg->mode == wippersnapper_pin_v1_Mode_MODE_ANALOG) {
    if (pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
      // Initialize analog io pin
      if (pinMsg->direction ==
          wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
        WS._analogIO->initAnalogInputPin(pin, pinMsg->period, pinMsg->pull,
                                         pinMsg->analog_read_mode);
      } else if (
          pinMsg->direction ==
          wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
        WS._analogIO->initAnalogOutputPin(pin);
      } else {
        WS_DEBUG_PRINTLN("ERROR: Unable to decode analog pin direction.")
        is_success = false;
      }
    } else if (
        pinMsg->request_type ==
        wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
      // Delete analog io pin
      WS._analogIO->deinitAnalogPin(pinMsg->direction, pin);
    } else {
      WS_DEBUG_PRINTLN("ERROR: Could not decode digital pin request type");
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Could not decode pin mode")
    is_success = false;
  }

  return is_success;
}

/*****************************************************************************/
/*!
    @brief  Decodes a repeated ConfigurePinRequests messages.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
    @returns  True if pin configuration decoded successfully, False otherwise.
*/
/*****************************************************************************/
bool cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field,
                          void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbDecodePinConfigMsg");

  // pb_decode the stream into a pinReqMessage
  wippersnapper_pin_v1_ConfigurePinRequest pinReqMsg =
      wippersnapper_pin_v1_ConfigurePinRequest_init_zero;
  if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequest_fields,
                 &pinReqMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
    is_success = false;
  }

  // Pass ConfigurePinRequest message
  if (!WS.configurePinRequest(&pinReqMsg)) {
    WS_DEBUG_PRINTLN("Unable to configure pin");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief  Decodes repeated PinEvents messages.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
*/
/**************************************************************************/
bool cbDecodePinEventMsg(pb_istream_t *stream, const pb_field_t *field,
                         void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbDecodePinEventMsg");

  // Decode stream into a PinEvent
  wippersnapper_pin_v1_PinEvent pinEventMsg =
      wippersnapper_pin_v1_PinEvent_init_zero;
  if (!pb_decode(stream, wippersnapper_pin_v1_PinEvent_fields, &pinEventMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode PinEvents")
    is_success = false;
  }

  char *pinName = pinEventMsg.pin_name + 1;
  if (pinEventMsg.pin_name[0] == 'D') { // digital pin event
    WS._digitalGPIO->digitalWriteSvc(atoi(pinName),
                                     atoi(pinEventMsg.pin_value));
  } else if (pinEventMsg.pin_name[0] == 'A') { // analog pin event
    // TODO
    WS_DEBUG_PRINTLN("ERROR: Analog PinEvent unimplemented!");
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unable to decode pin event name.");
    is_success = false;
  }

  return is_success;
}

// Decoding API

/**************************************************************************/
/*!
    @brief      Sets payload callbacks inside the signal message's
                submessage.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
*/
/**************************************************************************/
bool cbSignalMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbSignalMsg");

  pb_size_t arr_sz = field->array_size;
  WS_DEBUG_PRINT("Sub-messages found: ");
  WS_DEBUG_PRINTLN(arr_sz);

  if (field->tag ==
      wippersnapper_signal_v1_CreateSignalRequest_pin_configs_tag) {
    WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Configuration");
    // array to store the decoded CreateSignalRequests data
    wippersnapper_pin_v1_ConfigurePinRequests msg =
        wippersnapper_pin_v1_ConfigurePinRequests_init_zero;
    // set up callback
    msg.list.funcs.decode = cbDecodePinConfigMsg;
    msg.list.arg = field->pData;
    // decode each ConfigurePinRequest sub-message
    if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequests_fields,
                   &msg)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSign2alRequest")
      is_success = false;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_CreateSignalRequest_pin_events_tag) {
    WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Event");
    // array to store the decoded PinEvents data
    wippersnapper_pin_v1_PinEvents msg =
        wippersnapper_pin_v1_PinEvents_init_zero;
    // set up callback
    msg.list.funcs.decode = cbDecodePinEventMsg;
    msg.list.arg = field->pData;
    // decode each PinEvents sub-message
    if (!pb_decode(stream, wippersnapper_pin_v1_PinEvents_fields, &msg)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSign2alRequest")
      is_success = false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Unexpected signal msg tag.");
  }

  // once this is returned, pb_dec_submessage()
  // decodes the submessage contents.
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Decodes a signal buffer protobuf message.
        NOTE: Should be executed in-order after a new _buffer is recieved.
    @param    encodedSignalMsg
              Encoded signal message.
    @return   true if successfully decoded signal message, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper::decodeSignalMsg(
    wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("decodeSignalMsg");

  /* Set up the payload callback, which will set up the callbacks for
  each oneof payload field once the field tag is known */
  encodedSignalMsg->cb_payload.funcs.decode = cbSignalMsg;

  // decode the CreateSignalRequest, calls cbSignalMessage and assoc. callbacks
  pb_istream_t stream = pb_istream_from_buffer(WS._buffer, WS.bufSize);
  if (!pb_decode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                 encodedSignalMsg)) {
    WS_DEBUG_PRINTLN(
        "ERROR (decodeSignalMsg):, Could not decode CreateSignalRequest")
    is_success = false;
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Called when signal topic receives a new message. Fills
                shared buffer with data from payload.
*/
/**************************************************************************/
void cbSignalTopic(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("cbSignalTopic: New Msg on Signal Topic");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WS._buffer, 0, sizeof(WS._buffer));
  // copy data to buffer
  memcpy(WS._buffer, data, len);
  WS.bufSize = len;

  // Empty struct for storing the signal message
  WS._incomingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

  // Attempt to decode a signal message
  if (!WS.decodeSignalMsg(&WS._incomingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to decode signal message");
  }
}

/**************************************************************************/
/*!
    @brief    Called when broker responds to a device's publish across
                the registration topic.
*/
/**************************************************************************/
void cbRegistrationStatus(char *data, uint16_t len) {
  WS._registerBoard->decodeRegMsg(data, len);
}

/**************************************************************************/
/*!
    @brief    Attempts to re-connect to the MQTT broker, retries with
                an exponential backoff + jitter interval.
*/
/**************************************************************************/
void retryMQTTConnection() {
  // MQTT broker's connack return code
  int8_t rc;
  // amount of times we've attempted to re-connect to IO's MQTT broker
  int retries = 0;
  // maximum backoff time, in millis
  double maxBackoff = 60000;
  // current backoff time, in millis
  double backoff;
  // randomized jitter to prevent multi-client collisions
  long jitter;

  bool notConnected = true;
  while (notConnected == true) {
    WS_DEBUG_PRINTLN("Retrying connection...");
    // attempt reconnection, save return code (rc)
    rc = WS._mqtt->connect(WS._username, WS._key);
    switch (rc) {
    case WS_MQTT_CONNECTED:
      WS_DEBUG_PRINTLN("Re-connected to IO MQTT!");
      notConnected = false;
      break;
    case WS_MQTT_INVALID_PROTOCOL:
      WS_DEBUG_PRINTLN("Invalid MQTT protocol");
      break;
    case WS_MQTT_INVALID_CID:
      WS_DEBUG_PRINTLN("client ID rejected");
      break;
    case WS_MQTT_SERVICE_UNAVALIABLE:
      WS_DEBUG_PRINTLN("MQTT service unavailable");
      break;
    case WS_MQTT_INVALID_USER_PASS:
      WS_DEBUG_PRINTLN("malformed user/pass");
      break;
    case WS_MQTT_UNAUTHORIZED:
      WS_DEBUG_PRINTLN("unauthorized");
      break;
    case WS_MQTT_THROTTLED:
      WS_DEBUG_PRINTLN("ERROR: Throttled");
      break;
    case WS_MQTT_BANNED:
      WS_DEBUG_PRINTLN("ERROR: Temporarily banned");
      break;
    default:
      break;
    }
    retries++;
    if (notConnected) {
      WS_DEBUG_PRINTLN("Not connected, delaying...");
      // calculate a jitter value btween 0ms and 100ms
      jitter = random(0, 100);
      // calculate exponential backoff w/jitter
      backoff = (pow(2, retries) * 1000) + jitter;
      backoff = min(backoff, maxBackoff);
      WS_DEBUG_PRINT("Delaying for ");
      WS_DEBUG_PRINT(backoff);
      WS_DEBUG_PRINTLN("ms...");
      // delay for backoff millis
      delay(backoff);
    } else {
      WS_DEBUG_PRINTLN("Connected to MQTT broker!")
      // reset backoff param and retries
      backoff = 0;
    }
  }
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /error special topic.
*/
/**************************************************************************/
void cbErrorTopic(char *errorData, uint16_t len) {
  WS_DEBUG_PRINT("IO Ban Error: ");
  WS_DEBUG_PRINTLN(errorData);
  // Disconnect client from broker
  WS_DEBUG_PRINT("Disconnecting from MQTT..");
  if (!WS._mqtt->disconnect()) {
    WS_DEBUG_PRINTLN("ERROR: Unable to disconnect from MQTT broker!");
  }
  // attempt to re-establish a MQTT connection
  retryMQTTConnection();
}

/**************************************************************************/
/*!
    @brief    Called when client receives a message published across the
                Adafruit IO MQTT /throttle special topic. Delays until
                throttle is released.
*/
/**************************************************************************/
void cbThrottleTopic(char *throttleData, uint16_t len) {
  WS_DEBUG_PRINT("IO Throttle Error: ");
  WS_DEBUG_PRINTLN(throttleData);
  // Parse out # of seconds from throttle error message
  WS.throttleMessage = strtok(throttleData, ",");
  WS.throttleMessage = strtok(NULL, " ");
  // Convert to millis for delay
  WS.throttleTime = atoi(WS.throttleMessage) * 1000;
  WS_DEBUG_PRINT("Delaying for: ");
  WS_DEBUG_PRINTLN(WS.throttleTime);
  // Calculate amount of times to delay WS_KEEPALIVE_INTERVAL_MS
  double throttleTimes = WS.throttleTime / WS_KEEPALIVE_INTERVAL_MS;
  // round to nearest millis to prevent delaying for less time than req'd.
  throttleTimes = ceil(throttleTimes);
  for (int i = 0; i < (int)throttleTimes; i++) {
    WS_DEBUG_PRINTLN("Delaying...")
    delay(WS_KEEPALIVE_INTERVAL_MS);
    WS._mqtt->ping(); // keep the connection active
  }
}

/**************************************************************************/
/*!
    @brief    Builds MQTT topics for handling errors returned from the
                Adafruit IO broker.
    @returns  True if memory for error topics allocated successfully,
                False otherwise.
*/
/**************************************************************************/
bool Wippersnapper::buildErrorTopics() {
  bool is_success = true;
  // dynamically allocate memory for err topic
  WS._err_topic = (char *)malloc(
      sizeof(char) * (strlen(WS._username) + strlen(TOPIC_IO_ERRORS) + 1));

  if (WS._err_topic) { // build error topic
    strcpy(WS._err_topic, WS._username);
    strcat(WS._err_topic, TOPIC_IO_ERRORS);
  } else { // malloc failed
    WS._err_topic = 0;
    is_success = false;
  }

  // dynamically allocate memory for throttle topic
  WS._throttle_topic = (char *)malloc(
      sizeof(char) * (strlen(WS._username) + strlen(TOPIC_IO_THROTTLE) + 1));

  if (WS._throttle_topic) { // build throttle topic
    strcpy(WS._throttle_topic, WS._username);
    strcat(WS._throttle_topic, TOPIC_IO_THROTTLE);
  } else { // malloc failed
    WS._throttle_topic = 0;
    is_success = false;
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Subscribes to user-specific Adafruit IO MQTT topics
*/
/**************************************************************************/
void Wippersnapper::subscribeErrorTopics() {
  // Subscribe to error topic
  _err_sub = new Adafruit_MQTT_Subscribe(WS._mqtt, WS._err_topic);
  WS._mqtt->subscribe(_err_sub);
  _err_sub->setCallback(cbErrorTopic);

  // Subscribe to throttle topic
  _throttle_sub = new Adafruit_MQTT_Subscribe(WS._mqtt, WS._throttle_topic);
  WS._mqtt->subscribe(_throttle_sub);
  _throttle_sub->setCallback(cbThrottleTopic);
}

/**************************************************************************/
/*!
    @brief    Generates device-specific Wippersnapper control topics.
    @returns  True if memory for control topics allocated successfully,
                False otherwise.
*/
/**************************************************************************/
bool Wippersnapper::buildWSTopics() {
  bool is_success = true;
  // Get UID from the network iface
  setUID();
  // Move the top 3 bytes from the UID
  for (int i = 5; i > 2; i--) {
    WS._uid[6 - 1 - i] = WS._uid[i];
  }
  snprintf(WS.sUID, sizeof(WS.sUID), "%02d%02d%02d", WS._uid[0], WS._uid[1],
           WS._uid[2]);

  // Get board ID from _Boards.h
  WS._boardId = BOARD_ID;

  // Set client UID
  _device_uid = (char *)malloc(sizeof(char) + strlen("io-wipper-") +
                               strlen(WS._boardId) + strlen(WS.sUID));
  strcpy(_device_uid, "io-wipper-");
  strcat(_device_uid, WS._boardId);
  strcat(_device_uid, WS.sUID);

  // Create MQTT client object
  setupMQTTClient(_device_uid);

  // Global registration topic
  WS._topic_description =
      (char *)malloc(sizeof(char) * strlen(WS._username) + strlen("/wprsnpr") +
                     strlen(TOPIC_DESCRIPTION) + strlen("status") + 1);

  // Registration status topic
  WS._topic_description_status = (char *)malloc(
      sizeof(char) * strlen(WS._username) + +strlen("/wprsnpr/") +
      strlen(_device_uid) + strlen(TOPIC_DESCRIPTION) + strlen("status") +
      strlen("broker") + 1);

  // Topic for signals from device to broker
  WS._topic_signal_device = (char *)malloc(
      sizeof(char) * strlen(WS._username) + +strlen("/") + strlen(_device_uid) +
      strlen("/wprsnpr/") + strlen(TOPIC_SIGNALS) + strlen("device") + 1);

  // Topic for signals from broker to device
  WS._topic_signal_brkr = (char *)malloc(
      sizeof(char) * strlen(WS._username) + +strlen("/") + strlen(_device_uid) +
      strlen("/wprsnpr/") + strlen(TOPIC_SIGNALS) + strlen("broker") + 1);

  // Create global registration topic
  if (WS._topic_description) {
    strcpy(WS._topic_description, WS._username);
    strcat(WS._topic_description, "/wprsnpr");
    strcat(WS._topic_description, TOPIC_DESCRIPTION);
    strcat(WS._topic_description, "status");
  } else { // malloc failed
    WS._topic_description = 0;
    is_success = false;
  }

  // Create registration status topic
  if (WS._topic_description_status) {
    strcpy(WS._topic_description_status, WS._username);
    strcat(WS._topic_description_status, "/wprsnpr/");
    strcat(WS._topic_description_status, _device_uid);
    strcat(WS._topic_description_status, TOPIC_DESCRIPTION);
    strcat(WS._topic_description_status, "status");
    strcat(WS._topic_description_status, "/broker");
  } else { // malloc failed
    WS._topic_description_status = 0;
    is_success = false;
  }

  // Create device-to-broker signal topic
  if (WS._topic_signal_device) {
    strcpy(WS._topic_signal_device, WS._username);
    strcat(WS._topic_signal_device, "/wprsnpr/");
    strcat(WS._topic_signal_device, _device_uid);
    strcat(WS._topic_signal_device, TOPIC_SIGNALS);
    strcat(WS._topic_signal_device, "device");
  } else { // malloc failed
    WS._topic_signal_device = 0;
    is_success = false;
  }

  // Create broker-to-device signal topic
  if (WS._topic_signal_brkr) {
    strcpy(WS._topic_signal_brkr, WS._username);
    strcat(WS._topic_signal_brkr, "/wprsnpr/");
    strcat(WS._topic_signal_brkr, _device_uid);
    strcat(WS._topic_signal_brkr, TOPIC_SIGNALS);
    strcat(WS._topic_signal_brkr, "broker");
  } else { // malloc failed
    WS._topic_signal_brkr = 0;
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief    Subscribes to device-specific MQTT control topics.
*/
/**************************************************************************/
void Wippersnapper::subscribeWSTopics() {
  // Subscribe to signal topic
  _topic_signal_brkr_sub =
      new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_signal_brkr, 1);
  WS._mqtt->subscribe(_topic_signal_brkr_sub);
  _topic_signal_brkr_sub->setCallback(cbSignalTopic);

  // Subscribe to registration status topic
  _topic_description_sub =
      new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_description_status, 1);
  WS._mqtt->subscribe(_topic_description_sub);
  _topic_description_sub->setCallback(cbRegistrationStatus);
}

/**************************************************************************/
/*!
    @brief    Connects to Adafruit IO+ Wippersnapper broker.
*/
/**************************************************************************/
void Wippersnapper::connect() {
  WS_DEBUG_PRINTLN("connect()");

  statusLEDInit();
  setStatusLEDColor(LED_HW_INIT);

  _status = WS_IDLE;
  WS._boardStatus = WS_BOARD_DEF_IDLE;

  // Connect network interface
  WS_DEBUG_PRINTLN("Connecting to WiFi...");
  setStatusLEDColor(LED_NET_CONNECT);
  _connect();
  WS_DEBUG_PRINTLN("Connected!");

  // setup MQTT client
  setStatusLEDColor(LED_IO_CONNECT);
  // attempt to build Wippersnapper MQTT topics
  if (!buildWSTopics()) {
    WS_DEBUG_PRINTLN("Unable to allocate memory for Wippersnapper topics.")
    _disconnect();
    setStatusLEDColor(LED_ERROR);
    for (;;) {
      delay(1000);
    }
  }

  // Subscribe to wippersnapper topics
  subscribeWSTopics();

  // Attempt to build error MQTT topics
  if (!buildErrorTopics()) {
    WS_DEBUG_PRINTLN("Unable to allocate memory for error topics.")
    _disconnect();
    setStatusLEDColor(LED_ERROR);
    for (;;) {
      delay(1000);
    }
  }
  // Subscribe to error topics
  subscribeErrorTopics();

  // MQTT setup
  WS._mqtt->setKeepAliveInterval(WS_KEEPALIVE_INTERVAL);

  // Wait for connection to broker
  WS_DEBUG_PRINT("Connecting to Wippersnapper MQTT...");
  while (status() < WS_CONNECTED) {
    WS_DEBUG_PRINT(".");
    delay(500);
  }
  WS_DEBUG_PRINTLN("MQTT Connection Established!");

  // Register hardware with Wippersnapper
  WS_DEBUG_PRINTLN("Registering Board...")
  setStatusLEDColor(LED_IO_REGISTER_HW);
  if (!registerBoard(10)) {
    WS_DEBUG_PRINTLN("Unable to register board with Wippersnapper.");
    setStatusLEDColor(LED_ERROR);
    for (;;) {
      delay(1000);
    }
  }

  WS_DEBUG_PRINTLN("Registered board with Wippersnapper.");
  statusLEDBlink(WS_LED_STATUS_CONNECTED);
  statusLEDDeinit();
}

/**************************************************************************/
/*!
    @brief    Disconnects from Adafruit IO+ Wippersnapper.
*/
/**************************************************************************/
void Wippersnapper::disconnect() { _disconnect(); }

// Concrete class definition for abstract classes

/****************************************************************************/
/*!
    @brief    Connects to wireless network.
*/
/****************************************************************************/
void Wippersnapper::_connect() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Disconnect Wippersnapper MQTT session and network.
*/
/****************************************************************************/
void Wippersnapper::_disconnect() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets the network interface's unique identifer, typically the
              MAC address.
*/
/****************************************************************************/
void Wippersnapper::setUID() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets up the MQTT client session.
    @param    clientID
              A unique client identifier string.
*/
/****************************************************************************/
void Wippersnapper::setupMQTTClient(const char *clientID) {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Returns the network's connection status
    @returns  Network status as ws_status_t.
*/
/****************************************************************************/
ws_status_t Wippersnapper::networkStatus() {
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
void Wippersnapper::set_ssid_pass(const char *ssid, const char *ssidPassword) {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/****************************************************************************/
/*!
    @brief    Sets the device's wireless network credentials from the
              secrets.json configuration file.
*/
/****************************************************************************/
void Wippersnapper::set_ssid_pass() {
  WS_DEBUG_PRINTLN("ERROR: Please define a network interface!");
}

/**************************************************************************/
/*!
    @brief    Checks and handles network interface connection.
    @returns  Network status as ws_status_t.
*/
/**************************************************************************/
ws_status_t Wippersnapper::checkNetworkConnection() {
  if (status() < WS_NET_CONNECTED) {
    WS_DEBUG_PRINTLN("WiFi connection failed out, reconnecting...");
    unsigned long startRetry = millis();
    connect();
    while (status() < WS_CONNECTED) { // return an error on timeout
      if (millis() - startRetry > 60000) {
        return status();
      }
      delay(500);
    }
  }
  return status();
}

/**************************************************************************/
/*!
    @brief    Handles MQTT connection.
    @param    timeStart
                The time which this function was called, in milliseconds.
    @returns  Network status, as ws_status_t.
*/
/**************************************************************************/
ws_status_t Wippersnapper::checkMQTTConnection(uint32_t timeStart) {
  // Return quickly
  if (mqttStatus() == WS_CONNECTED) {
    return status();
  }

  // loop until we have a connection
  // mqttStatus() will try to reconnect before returning
  while (mqttStatus() != WS_CONNECTED && millis() - timeStart < 60000) {
  }
  if (mqttStatus() != WS_CONNECTED) {
    WS_DEBUG_PRINTLN("ERROR: Disconnected from MQTT!");
  }
  return status();
}

/**************************************************************************/
/*!
    @brief    Pings MQTT broker.
*/
/**************************************************************************/
void Wippersnapper::ping() { WS._mqtt->ping(); }

/****************************************************************************/
/*!
    @brief    Handles MQTT messages on signal topic until timeout.
    @param    outgoingSignalMsg
                Empty signal message struct.
    @param    pinMode
                Pin's input type.
    @param    pinName
                Name of pin.
    @param    pinVal
                Value of pin.
    @returns  True if pinEvent message encoded successfully, false otherwise.
*/
/****************************************************************************/
bool Wippersnapper::encodePinEvent(
    wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
    wippersnapper_pin_v1_Mode pinMode, uint8_t pinName, int pinVal) {
  bool is_success = true;
  outgoingSignalMsg->which_payload =
      wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
  // fill the pin_event message
  outgoingSignalMsg->payload.pin_event.mode = pinMode;
  sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "D%d", pinName);
  sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%d", pinVal);

  // Encode signal message
  pb_ostream_t stream =
      pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
  if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                 outgoingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief    Processes incoming commands and handles network connection.
    @returns  Network status, as ws_status_t.
*/
/**************************************************************************/
ws_status_t Wippersnapper::run() {
  // WS_DEBUG_PRINTLN("exec::run()");
  uint32_t curTime = millis();

  // Handle network connection
  checkNetworkConnection();
  // Handle MQTT connection
  checkMQTTConnection(curTime);

  // Process all incoming packets from Wippersnapper MQTT Broker
  WS._mqtt->processPackets(10);

  // Process digital inputs, digitalGPIO module
  WS._digitalGPIO->processDigitalInputs();

  // Process analog inputs
  WS._analogIO->processAnalogInputs();

  return status();
}

/**************************************************************************/
/*!
    @brief    Sends board description message to Wippersnapper
    @param    retries
              Amount of times to retry registration process.
    @returns  True if successful, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper::registerBoard(uint8_t retries = 10) {
  WS_DEBUG_PRINTLN("registerBoard()");
  // Create new board
  _registerBoard = new Wippersnapper_Registration();
  // Run the FSM for the registration process
  return _registerBoard->processRegistration();
}

/**************************************************************************/
/*!
    @brief    Returns the network status.
    @return   Wippersnapper network status.
*/
/**************************************************************************/
ws_status_t Wippersnapper::status() {
  ws_status_t net_status = networkStatus();

  // if we aren't connected, return network status
  if (net_status != WS_NET_CONNECTED) {
    _status = net_status;
    return _status;
  }

  // check mqtt status and return
  _status = mqttStatus();
  return _status;
}

/**************************************************************************/
/*!
    @brief    Returns the board definition status
    @return   Wippersnapper board definition status
*/
/**************************************************************************/
ws_board_status_t Wippersnapper::getBoardStatus() { return WS._boardStatus; }

/**************************************************************************/
/*!
    @brief    Checks connection status with Adafruit IO's MQTT broker.
    @return   True if connected, otherwise False.
*/
/**************************************************************************/
ws_status_t Wippersnapper::mqttStatus() {
  // if the connection failed,
  // return so we don't hammer IO
  if (_status == WS_CONNECT_FAILED) {
    WS_DEBUG_PRINT("mqttStatus() failed to connect");
    WS_DEBUG_PRINTLN(WS._mqtt->connectErrorString(_status));
    setStatusLEDColor(LED_ERROR);
    return _status;
  }

  if (WS._mqtt->connected()) {
    // ping within keepalive to keep connection open
    if (millis() > (_prv_ping + WS_KEEPALIVE_INTERVAL_MS)) {
      ping();
      _prv_ping = millis();
    }
    // blink status LED every STATUS_LED_KAT_BLINK_TIME millis
    if (millis() > (_prvKATBlink + STATUS_LED_KAT_BLINK_TIME)) {
      if (!statusLEDInit()) {
        WS_DEBUG_PRINTLN("Can not blink, status-LED in use");
      } else {
        statusLEDBlink(WS_LED_STATUS_KAT);
        statusLEDDeinit();
      }
      _prvKATBlink = millis();
    }
    return WS_CONNECTED;
  }

  // prevent fast reconnect attempts, except for the first time through
  if (_last_mqtt_connect == 0 ||
      millis() - _last_mqtt_connect > WS_KEEPALIVE_INTERVAL_MS) {
    _last_mqtt_connect = millis();
    switch (WS._mqtt->connect(WS._username, WS._key)) {
    case 0:
      return WS_CONNECTED;
    case 1: // invalid mqtt protocol
    case 2: // client id rejected
    case 4: // malformed user/pass
    case 5: // unauthorized
      return WS_CONNECT_FAILED;
    case 3: // mqtt service unavailable
    case 6: // throttled
    case 7: // banned -> all MQTT bans are temporary, so eventual retry is
            // permitted
            // delay to prevent fast reconnects and fast returns (backward
            // compatibility)
      delay(60000);
      return WS_DISCONNECTED;
    default:
      return WS_DISCONNECTED;
    }
  }
  return WS_DISCONNECTED;
}