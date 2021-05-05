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
 * This library depends on <a href="https://github.com/adafruit/Adafruit_Sensor">
 * Adafruit_Sensor</a> being present on your system. Please make sure you have
 * installed the latest version before using this library.
 *
 * @section author Author
 *
 * Written by Brent Rubell for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "Wippersnapper.h"

Wippersnapper WS;

Wippersnapper::Wippersnapper() {
    _mqtt = 0;     // MQTT Client object

    // IO creds
    _username = 0;
    _key = 0;

    // Reserved MQTT Topics
    _topic_description = 0;
    _topic_description_status = 0;
    _topic_signal_device = 0;
    _topic_signal_brkr = 0;
};

/**************************************************************************/
/*!
    @brief    Wippersnapper destructor
*/
/**************************************************************************/
Wippersnapper::~Wippersnapper() {
    // re-allocate topics
    free(_topic_description);
    free(_topic_signal_device);
    free(_topic_signal_brkr);
}

void Wippersnapper::set_user_key(const char *aio_username, const char *aio_key) {
    _username = aio_username;
    _key = aio_key;
}

// Decoders //
/****************************************************************************/
/*!
    @brief    Configures a pin according to a 
                wippersnapper_pin_v1_ConfigurePinRequest message.
    @param    pinMsg
              Pointer to a wippersnapper_pin_v1_ConfigurePinRequest message.
*/
/****************************************************************************/
bool Wippersnapper::configurePinRequest(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg) {
    WS_DEBUG_PRINTLN("configurePinRequest");

    bool is_success = true;
    char* pinName = pinMsg->pin_name + 1;
    int pin = atoi(pinName);

    // Decode pin mode
    if (pinMsg->mode == wippersnapper_pin_v1_Mode_MODE_DIGITAL) {
        if (pinMsg->request_type == wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
            // Initialize GPIO pin
            WS._digitalGPIO->initDigitalPin(pinMsg->direction, pin, pinMsg->period);
        } else if (pinMsg->request_type == wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
            // Delete digital GPIO pin
            WS._digitalGPIO->deinitDigitalPin(pinMsg->direction, pin);
        } else {
            WS_DEBUG_PRINTLN("ERROR: Could not decode digital pin request type");
        }
    }

    else if (pinMsg->mode = wippersnapper_pin_v1_Mode_MODE_ANALOG) {
        if (pinMsg->request_type == wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
            // Initialize analog io pin
            if (pinMsg->direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
                WS._analogIO->initAnalogInputPin(pin, pinMsg->period, pinMsg->pull, pinMsg->analog_read_mode);
            }
            else if (pinMsg->direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
                WS._analogIO->initAnalogOutputPin(pin);
            } else {
                WS_DEBUG_PRINTLN("ERROR: Unable to decode analog pin direction.")
                is_success = false;
            }
        } else if (pinMsg->request_type == wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
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

/**************************************************************************/
/*!
    @brief  Decodes a repeated ConfigurePinRequests messages.
*/
/**************************************************************************/
bool cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("cbDecodePinConfigMsg");

    // pb_decode the stream into a pinReqMessage
    wippersnapper_pin_v1_ConfigurePinRequest pinReqMsg = wippersnapper_pin_v1_ConfigurePinRequest_init_zero;
    if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequest_fields, &pinReqMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
        is_success = false;
    }

    // Pass ConfigurePinRequest message
    if (! WS.configurePinRequest(&pinReqMsg)){
        WS_DEBUG_PRINTLN("Unable to configure pin");
        is_success = false;
    }

    return is_success;
}


/**************************************************************************/
/*!
    @brief  Decodes repeated PinEvents messages.
*/
/**************************************************************************/
bool cbDecodePinEventMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("cbDecodePinEventMsg");

    // Decode stream into a PinEvent
    wippersnapper_pin_v1_PinEvent pinEventMsg = wippersnapper_pin_v1_PinEvent_init_zero;
    if (!pb_decode(stream, wippersnapper_pin_v1_PinEvent_fields, &pinEventMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Could not decode PinEvents")
        is_success = false;
    }

    char* pinName = pinEventMsg.pin_name + 1;
    if (pinEventMsg.pin_name[0] == 'D') { // digital pin event
        WS._digitalGPIO->digitalWriteSvc(atoi(pinName), atoi(pinEventMsg.pin_value));
    }
    else if (pinEventMsg.pin_name[0] == 'A') { // analog pin event
        // TODO
        WS_DEBUG_PRINTLN("ERROR: Analog PinEvent unimplemented!");
    }
    else {
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
*/
/**************************************************************************/
bool cbSignalMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("cbSignalMsg");

    pb_size_t arr_sz = field->array_size;
    WS_DEBUG_PRINT("Sub-messages found: "); WS_DEBUG_PRINTLN(arr_sz);

    if (field->tag == wippersnapper_signal_v1_CreateSignalRequest_pin_configs_tag) {
        WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Configuration");
        // array to store the decoded CreateSignalRequests data
        wippersnapper_pin_v1_ConfigurePinRequests msg = wippersnapper_pin_v1_ConfigurePinRequests_init_zero;
        // set up callback
        msg.list.funcs.decode = cbDecodePinConfigMsg;
        msg.list.arg = field->pData;
        // decode each ConfigurePinRequest sub-message
        if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequests_fields, &msg)) {
            WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSign2alRequest")
            is_success = false;
        }
    }
    else if (field->tag == wippersnapper_signal_v1_CreateSignalRequest_pin_events_tag) {
        WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Event");
        // array to store the decoded PinEvents data
        wippersnapper_pin_v1_PinEvents msg = wippersnapper_pin_v1_PinEvents_init_zero;
        // set up callback
        msg.list.funcs.decode = cbDecodePinEventMsg;
        msg.list.arg = field->pData;
        // decode each PinEvents sub-message
        if (!pb_decode(stream, wippersnapper_pin_v1_PinEvents_fields, &msg)) {
            WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSign2alRequest")
            is_success = false;
        }
    }
    else {
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
bool Wippersnapper::decodeSignalMsg(wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("decodeSignalMsg");

    /* Set up the payload callback, which will set up the callbacks for
    each oneof payload field once the field tag is known */
    encodedSignalMsg->cb_payload.funcs.decode = cbSignalMsg;

    // decode the CreateSignalRequest, calls cbSignalMessage and assoc. callbacks
    pb_istream_t stream = pb_istream_from_buffer(WS._buffer, WS.bufSize);
    if (!pb_decode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields, encodedSignalMsg)) {
        WS_DEBUG_PRINTLN("ERROR (decodeSignalMsg):, Could not decode CreateSignalRequest")
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
    WS_DEBUG_PRINTLN("-> Message on Signal Topic");
    WS_DEBUG_PRINT(len);WS_DEBUG_PRINTLN(" bytes.");
    // zero-out current buffer
    memset(WS._buffer, 0, sizeof(WS._buffer));
    // copy data to buffer
    memcpy(WS._buffer, data, len);
    WS.bufSize = len;
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
    @brief    Generates Wippersnapper feeds.
*/
/**************************************************************************/
void Wippersnapper::generate_subscribe_feeds() {

    // Set UID in the network iface
    setUID();
    // Move the top 3 bytes from the UID
    for (int i = 5; i > 2; i--) {
        WS._uid[6-1-i]  = WS._uid[i];
    }
    snprintf(WS.sUID, sizeof(WS.sUID), "%02d%02d%02d", WS._uid[0], WS._uid[1], WS._uid[2]);

    // Set board type and device UID
    WS._boardId = BOARD_ID;
    _device_uid = (char *)malloc(sizeof(char) + strlen("io-wipper-") + strlen(WS._boardId) + strlen(WS.sUID));
    strcpy(_device_uid, "io-wipper-");
    strcat(_device_uid, WS._boardId);
    strcat(_device_uid, WS.sUID);

    // Create MQTT client object
    setupMQTTClient(_device_uid);

    // Global registration topic
    WS._topic_description = (char *)malloc(sizeof(char) * strlen(_username) \
    + strlen("/wprsnpr") + strlen(TOPIC_DESCRIPTION) + strlen("status") + 1);

    // Registration status topic
    WS._topic_description_status = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/wprsnpr/") + strlen(_device_uid) + strlen(TOPIC_DESCRIPTION) + \
    strlen("status") + strlen("broker") + 1);

    // Topic for signals from device to broker
    WS._topic_signal_device = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/") + strlen(_device_uid) +  strlen("/wprsnpr/") + \
    strlen(TOPIC_SIGNALS) + strlen("device") + 1);

    // Topic for signals from broker to device
    WS._topic_signal_brkr = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/") + strlen(_device_uid) +  strlen("/wprsnpr/") + \
    strlen(TOPIC_SIGNALS) + strlen("broker") + 1);

    // Create global registration topic
    if (WS._topic_description) {
        strcpy(WS._topic_description, _username);
        strcat(WS._topic_description, "/wprsnpr");
        strcat(WS._topic_description, TOPIC_DESCRIPTION);
        strcat(WS._topic_description, "status");
    } else { // malloc failed
        WS._topic_description  = 0;
    }

    // Create registration status topic
    if (WS._topic_description_status) {
        strcpy(WS._topic_description_status, _username);
        strcat(WS._topic_description_status, "/wprsnpr/");
        strcat(WS._topic_description_status, _device_uid);
        strcat(WS._topic_description_status, TOPIC_DESCRIPTION);
        strcat(WS._topic_description_status, "status");
        strcat(WS._topic_description_status, "/broker");
    } else { // malloc failed
        WS._topic_description_status = 0;
    }

    // Create device-to-broker signal topic
    if (WS._topic_signal_device) {
        strcpy(WS._topic_signal_device, _username);
        strcat(WS._topic_signal_device, "/wprsnpr/");
        strcat(WS._topic_signal_device, _device_uid);
        strcat(WS._topic_signal_device, TOPIC_SIGNALS);
        strcat(WS._topic_signal_device, "device");
    } else { // malloc failed
        WS._topic_signal_device = 0;
    }

    // Create broker-to-device signal topic
    if (WS._topic_signal_brkr) {
        strcpy(WS._topic_signal_brkr, _username);
        strcat(WS._topic_signal_brkr, "/wprsnpr/");
        strcat(WS._topic_signal_brkr, _device_uid);
        strcat(WS._topic_signal_brkr, TOPIC_SIGNALS);
        strcat(WS._topic_signal_brkr, "broker");
    } else { // malloc failed
        WS._topic_signal_brkr = 0;
    }

    // Subscribe to signal topic
    _topic_signal_brkr_sub = new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_signal_brkr);
    WS._mqtt->subscribe(_topic_signal_brkr_sub);
    _topic_signal_brkr_sub->setCallback(cbSignalTopic);

    // Subscribe to registration status topic
    WS_DEBUG_PRINTLN(WS._topic_description_status);
    _topic_description_sub = new Adafruit_MQTT_Subscribe(WS._mqtt, WS._topic_description_status);
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

    _status = WS_IDLE;
    WS._boardStatus = WS_BOARD_DEF_IDLE;

    // Connect network interface
    WS_DEBUG_PRINT("Connecting to WiFi...");
    _connect();
    WS_DEBUG_PRINTLN("Connected!");

    // Generate and subscribe to MQTT feeds
    generate_subscribe_feeds();


    // Wait for connection to broker
    WS_DEBUG_PRINT("Connecting to Wippersnapper MQTT...");

    while (status() < WS_CONNECTED) {
        WS_DEBUG_PRINT(".");
        delay(500);
    }
    WS_DEBUG_PRINTLN("Connected!");


    WS_DEBUG_PRINTLN("Registering Board...")
    if (!registerBoard(10)) {
        WS_DEBUG_PRINTLN("Unable to register board with Wippersnapper.");
        for(;;) {
            pixels.setPixelColor(0, pixels.Color(255, 0, 255));
            pixels.show();
            delay(1000);
            pixels.setPixelColor(0, pixels.Color(0, 0, 0));
            pixels.show();
        }
    }
    WS_DEBUG_PRINTLN("Registered board with Wippersnapper.");


    #ifdef STATUS_NEOPIXEL
        pixels.setPixelColor(0, pixels.Color(0, 255, 0));
        pixels.show();
        delay(500);
        pixels.setPixelColor(0, pixels.Color(0, 0, 0));
        pixels.show();
    #else
        digitalWrite(STATUS_LED_PIN, 0);
        delay(500);
        digitalWrite(STATUS_LED_PIN, 1);
        delay(500);
        digitalWrite(STATUS_LED_PIN, 0);
        // de-init pin
        WS._digitalGPIO->deinitDigitalPin(STATUS_LED_PIN);
    #endif
}

/**************************************************************************/
/*!
    @brief    Disconnects from Adafruit IO+ Wippersnapper.
*/
/**************************************************************************/
void Wippersnapper::disconnect() {
    _disconnect();
}

// Concrete class definition for abstract classes
void Wippersnapper::_connect() {
    WS_DEBUG_PRINTLN("ERROR: Please define a network interface");
}

void Wippersnapper::_disconnect() {
    WS_DEBUG_PRINTLN("ERROR: Please define a network interface");
}

void Wippersnapper::setUID() {
    WS_DEBUG_PRINTLN("ERROR: Please define a network interface");
}

void Wippersnapper::setupMQTTClient(char const*) {
    WS_DEBUG_PRINTLN("ERROR: Please define a network interface");
}
ws_status_t Wippersnapper::networkStatus() {
    WS_DEBUG_PRINTLN("ERROR: Please define a network interface");
}

/**************************************************************************/
/*!
    @brief    Checks and handles network interface connection.
*/
/**************************************************************************/
ws_status_t Wippersnapper::checkNetworkConnection(uint32_t timeStart) {
    if (status() < WS_NET_CONNECTED) {
        WS_DEBUG_PRINTLN("connection failed, reconnecting...");
        unsigned long startRetry = millis();
        while (status() < WS_CONNECTED) { // return an error on timeout
            if (millis() - startRetry > 5000) {
                return status();
            }
            delay(500);
        }

    }
    // TODO: Check for status, re-send registration message
    return status();
}

/**************************************************************************/
/*!
    @brief    Handles MQTT connection.
*/
/**************************************************************************/
ws_status_t Wippersnapper::checkMQTTConnection(uint32_t timeStart) {
    // Check network connection
    if (mqttStatus() != WS_CONNECTED) {
        return status();
    }
    // Ping if > keepAlive interval
    if (millis() > (_prv_ping + WS_KEEPALIVE_INTERVAL)) {
        WS._mqtt->ping();
        _prv_ping = millis();
    }

    return status();
}

/**************************************************************************/
/*!
    @brief    Processes MQTT messages across signal topic.
    @param    timeout
                timeout in milliseconds.
*/
/**************************************************************************/
bool Wippersnapper::processSignalMessages(int16_t timeout) {
    WS._mqtt->processPackets(timeout);

    if (WS._buffer[0] != 0) { // check if buffer filled by signal topic callback
        WS_DEBUG_PRINTLN("-> Payload Data:");
        for (int i = 0; i < sizeof(WS._buffer); i++) {
            WS_DEBUG_PRINT(WS._buffer[i]);
        }
        WS_DEBUG_PRINTLN("");

        // Empty struct for storing the signal message
        _incomingSignalMsg = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

        // Attempt to decode a signal message
        if (! decodeSignalMsg(&_incomingSignalMsg)) {
            WS_DEBUG_PRINTLN("ERROR: Failed to decode signal message");
            return false;
        }
        memset(WS._buffer, 0, sizeof(WS._buffer));
    }

    return true;
}

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
bool Wippersnapper::encodePinEvent(wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg, wippersnapper_pin_v1_Mode pinMode, uint8_t pinName, int pinVal) {
    bool is_success = true;
    outgoingSignalMsg->which_payload = wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
    // fill the pin_event message
    outgoingSignalMsg->payload.pin_event.mode = pinMode;
    sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "D%d", pinName);
    sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%d", pinVal);

    // Encode signal message
    pb_ostream_t stream = pb_ostream_from_buffer(WS._buffer_outgoing, sizeof(WS._buffer_outgoing));
    if (!pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields, outgoingSignalMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
        is_success = false;
    }

    return is_success;
}

/**************************************************************************/
/*!
    @brief    Processes incoming commands and handles network connection.
*/
/**************************************************************************/
ws_status_t Wippersnapper::run() {
    uint32_t curTime = millis();
    // Check network connection
    checkNetworkConnection(curTime); // TODO: handle this better
    // Check and handle MQTT connection
    checkMQTTConnection(curTime); // TODO: handle this better

    // Process digital inputs, digitalGPIO module
    WS._digitalGPIO->processDigitalInputs();

    // Process analog inputs
    WS._analogIO->processAnalogInputs();

    // Process all incoming packets from Wippersnapper MQTT Broker
    processSignalMessages(100);

    return status();
}

/**************************************************************************/
/*!
    @brief    Sends board description message to Wippersnapper
*/
/**************************************************************************/
bool Wippersnapper::registerBoard(uint8_t retries=10) {
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
ws_board_status_t Wippersnapper::getBoardStatus() {
    return WS._boardStatus;
}

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
    return _status;
  }

  if (WS._mqtt->connected())
    return WS_CONNECTED;

  // prevent fast reconnect attempts, except for the first time through
  if (_last_mqtt_connect == 0 ||
      millis() - _last_mqtt_connect > WS_KEEPALIVE_INTERVAL) {
    _last_mqtt_connect = millis();
    switch (WS._mqtt->connect(_username, _key)) {
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
        delay(WS_KEEPALIVE_INTERVAL);
      return WS_DISCONNECTED;
    default:
      return WS_DISCONNECTED;
    }
  }
  return WS_DISCONNECTED;
}


/**************************************************************************/
/*!
    @brief    Provide status explanation strings.
    @return   A pointer to the status string, _status. _status is the BC status
   value
*/
/**************************************************************************/
const __FlashStringHelper *Wippersnapper::statusText() {
  switch (_status) {
    // CONNECTING
    case WS_IDLE:
        return F("Idle. Waiting for connect to be called...");
    case WS_NET_DISCONNECTED:
        return F("Network disconnected.");
    case WS_DISCONNECTED:
        return F("Disconnected from Wippersnapper.");
    // FAILURE
    case WS_NET_CONNECT_FAILED:
        return F("Network connection failed.");
    case WS_CONNECT_FAILED:
        return F("Wippersnapper connection failed.");
    case WS_FINGERPRINT_INVALID:
        return F("Wippersnapper SSL fingerprint verification failed.");
    case WS_AUTH_FAILED:
        return F("Wippersnapper authentication failed.");
    // SUCCESS
    case WS_NET_CONNECTED:
        return F("Network connected.");
    case WS_CONNECTED:
        return F("Wippersnapper connected.");
    case WS_CONNECTED_INSECURE:
        return F("Wippersnapper connected. **THIS CONNECTION IS INSECURE** SSL/TLS "
                "not supported for this platform.");
    case WS_FINGERPRINT_UNSUPPORTED:
        return F("Wippersnapper connected over SSL/TLS. Fingerprint verification "
                "unsupported.");
    case WS_FINGERPRINT_VALID:
        return F("Wippersnapper connected over SSL/TLS. Fingerprint valid.");
    default:
        return F("Unknown status code");
  }
}
