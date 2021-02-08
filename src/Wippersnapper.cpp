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

ws_board_status_t _boardStatus; // TODO: move to header

uint16_t Wippersnapper::bufSize;
uint8_t Wippersnapper::_buffer[128];
char Wippersnapper:: _value[45];
Wippersnapper::pinInfo Wippersnapper::ws_pinInfo;
/**************************************************************************/
/*!
    @brief    Instantiates the Wippersnapper client object.
    @param    aio_username
              Adafruit IO Username.
    @param    aio_key
              Adafruit IO Active Key.
*/
/**************************************************************************/
Wippersnapper::Wippersnapper(const char *aio_username, const char *aio_key) {
    _mqtt = 0;     // MQTT Client object

    _username = aio_username;
    _key = aio_key;

    // TODO: Remove!
    _deviceId = "myDevice"; // Adafruit IO+ device name
    _hw_vid = 0;       // Hardware's usb vendor id
    _hw_pid = 0;       // Hardware's usb product id

    // Reserved MQTT Topics //
    _topic_description = 0;
    _topic_description_status = 0;
    _topic_signal_device = 0;
    _topic_signal_brkr = 0;

    //_init();
}

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

/**************************************************************************/
/*!
    @brief This function is the core of the createSignalRequest encoding process.
          It handles the top-level pb_field_t array manually, in order to encode
          a correct field tag before the message. The pointer to MsgType_fields
          array is used as an unique identifier for the message type.
*/
/**************************************************************************/
bool Wippersnapper::encode_unionmessage(pb_ostream_t *stream, const pb_msgdesc_t *messagetype, void *message)
{
    pb_field_iter_t iter;

    if (!pb_field_iter_begin(&iter, wippersnapper_signal_v1_CreateSignalRequest_fields, message))
        return false;
    do
    {
        if (iter.submsg_desc == messagetype)
        {
            if (!pb_encode_tag_for_field(stream, &iter))
                return false;
            return pb_encode_submessage(stream, messagetype, message);
        }
    } while (pb_field_iter_next(&iter));
    return false;
}

/**************************************************************************/
/*!
    @brief    Executes when signal topic receives a new message. Fills
                shared buffer with data from payload.
*/
/**************************************************************************/
void Wippersnapper::cbSignalTopic(char *data, uint16_t len) {
    WS_DEBUG_PRINTLN("cbSignalTopic()");
    memcpy(_buffer, data, len);
    bufSize = len;
}

/****************************************************************************/
/*!
    @brief    Configures a pin according to a 
                wippersnapper_pin_v1_ConfigurePinRequest message.
    @param    pinMsg
              Pointer to a wippersnapper_pin_v1_ConfigurePinRequest message.
*/
/****************************************************************************/
bool Wippersnapper::configPinReq(wippersnapper_pin_v1_ConfigurePinRequest *pinMsg) {
    bool is_configured = true;
    WS_DEBUG_PRINTLN("configPinReq");

    // strip "a/d" circuitpython pin name prefix
    char* pinName = pinMsg->pin_name + 1;
    if (pinMsg->mode == wippersnapper_pin_v1_ConfigurePinRequest_Mode_MODE_DIGITAL) {
        // Configure a digital pin
        if (pinMsg->direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
            pinMode(atoi(pinName), pinMsg->direction);
            WS_DEBUG_PRINT("Configured digital output pin on ");WS_DEBUG_PRINTLN(pinName);
        }
        else if (pinMsg->direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
            WS_DEBUG_PRINTLN("Configuring digital input pin");
        }
        else {
            WS_DEBUG_PRINTLN("Unable to configure digital pin");
        }
    }
    else if (pinMsg->mode == wippersnapper_pin_v1_ConfigurePinRequest_Mode_MODE_ANALOG) {
        // Configure an analog pin
        if (pinMsg->direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_OUTPUT) {
            WS_DEBUG_PRINTLN("Configuring analog output pin");
        }
        else if (pinMsg->direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
            WS_DEBUG_PRINTLN("Configuring analog input pin");
        }
        else {
            WS_DEBUG_PRINTLN("Unable to configure digital pin");
        }
    }
    return is_configured;
}

/**************************************************************************/
/*!
    @brief  Decodes repeated ConfigurePinRequests messages.
*/
/**************************************************************************/
bool Wippersnapper::cbDecodePinConfigMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("cbDecodePinConfigMsg");

    // pb_decode the stream into a pinReqMessage
    wippersnapper_pin_v1_ConfigurePinRequest pinReqMsg = wippersnapper_pin_v1_ConfigurePinRequest_init_zero;
    if (!pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequest_fields, &pinReqMsg)) {
        WS_DEBUG_PRINTLN("L191 ERROR: Could not decode CreateSignalRequest")
        is_success = false;
    }

    // Configure physical pin
    if (! configPinReq(&pinReqMsg)){
        WS_DEBUG_PRINTLN("Unable to configure pin");
        is_success = false;
    }

    // TODO: Freeup struct members
    return is_success;
}

/****************************************************************************/
/*!
    @brief    Configures a pin according to a 
                PinEvent message.
    @param    pinEventMsg
              Pointer to a pinEvent message.
*/
/****************************************************************************/
bool Wippersnapper::writePinEvent(wippersnapper_pin_v1_PinEvent *pinEventMsg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("writePinEvent");

    char* pinName = pinEventMsg->pin_name + 1;
    if (pinEventMsg->pin_name[0] == 'D') {
        WS_DEBUG_PRINT("Digital Pin Event: Set ");WS_DEBUG_PRINT(pinName);
        WS_DEBUG_PRINT(" to ");WS_DEBUG_PRINTLN(pinEventMsg->pin_value);
        digitalWrite(atoi(pinName), atoi(pinEventMsg->pin_value));
    }
    else if (pinEventMsg->pin_name[0] == 'A') {
        WS_DEBUG_PRINT("Analog Pin Event: Set ");WS_DEBUG_PRINT(pinName);
        WS_DEBUG_PRINT(" to ");WS_DEBUG_PRINTLN(pinEventMsg->pin_value);
        #ifndef ARDUINO_ARCH_ESP32
            analogWrite(atoi(pinName), atoi(pinEventMsg->pin_value));
        #else
            WS_DEBUG_PRINTLN("ESP32 AnalogWrite not implemented yet.");
        #endif
    } else {
        WS_DEBUG_PRINTLN("Invalid pin event name");
        is_success = false;
    }
    return is_success;
}

/**************************************************************************/
/*!
    @brief  Decodes repeated PinEvents messages.
*/
/**************************************************************************/
bool Wippersnapper::cbDecodePinEventMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("cbDecodePinEventMsg");

    // Decode stream into a PinEvent
    wippersnapper_pin_v1_PinEvent pinEventMsg = wippersnapper_pin_v1_PinEvent_init_zero;
    if (!pb_decode(stream, wippersnapper_pin_v1_PinEvent_fields, &pinEventMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Could not decode PinEvents")
        is_success = false;
    }

    // Write to physical pin
    if (! writePinEvent(&pinEventMsg)){
        WS_DEBUG_PRINTLN("Unable to write to pin");
        is_success = false;
    }

    return is_success;
}

/**************************************************************************/
/*!
    @brief      Sets payload callbacks inside the signal message's
                submessage.
*/
/**************************************************************************/
bool Wippersnapper::cbSignalMsg(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    bool is_success = true;
    WS_DEBUG_PRINTLN("cbSignalMsg");

    pb_size_t arr_sz = field->array_size;
    WS_DEBUG_PRINT("Sub-messages found: "); WS_DEBUG_PRINTLN(arr_sz);

    if (field->tag == wippersnapper_signal_v1_CreateSignalRequest_pin_configs_tag) {
        WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Configuration");
        // array to store the decoded CreateSignalRequests data
        wippersnapper_pin_v1_ConfigurePinRequests msg = wippersnapper_pin_v1_ConfigurePinRequests_init_zero;
        // set up callback
        msg.list.funcs.decode = &Wippersnapper::cbDecodePinConfigMsg;
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
        msg.list.funcs.decode = &Wippersnapper::cbDecodePinEventMsg;
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
    encodedSignalMsg->cb_payload.funcs.decode = &Wippersnapper::cbSignalMsg;

    // decode the CreateSignalRequest, calls cbSignalMessage and assoc. callbacks
    pb_istream_t stream = pb_istream_from_buffer(_buffer, bufSize);
    if (!pb_decode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields, encodedSignalMsg)) {
        WS_DEBUG_PRINTLN("L285 ERROR: Could not decode CreateSignalRequest")
        is_success = false;
    }
    return is_success;
}

/**************************************************************************/
/*!
    @brief    Executes when the broker publishes a response to the
                client's board description message.
*/
/**************************************************************************/
void cbDescriptionStatus(char *data, uint16_t len) {
    uint8_t buffer[len];
    memcpy(buffer, data, len);

    // init. CreateDescriptionResponse message
    wippersnapper_description_v1_CreateDescriptionResponse message = wippersnapper_description_v1_CreateDescriptionResponse_init_zero;

    // create input stream for buffer
    pb_istream_t stream = pb_istream_from_buffer(buffer, len);
    // decode the stream
    if (!pb_decode(&stream, wippersnapper_description_v1_CreateDescriptionResponse_fields, &message)) {
        WS_DEBUG_PRINTLN("Error decoding description status message!");
    } else {    // set board status
        switch (message.response) {
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK:
                _boardStatus = WS_BOARD_DEF_OK;
                break;
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND:
                _boardStatus = WS_BOARD_DEF_INVALID;
                break;
            case wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED:
                _boardStatus = WS_BOARD_DEF_UNSPECIFIED;
                break;
            default:
                _boardStatus = WS_BOARD_DEF_UNSPECIFIED;
        }
    }

    WS_DEBUG_PRINTLN("\nSuccessfully checked in, waiting for commands...")
}

/**************************************************************************/
/*!
    @brief    Generates Wippersnapper feeds.
*/
/**************************************************************************/
void Wippersnapper::generate_feeds() {

    // Check and set network iface UID
    setUID();

    // Move the top 3 bytes from the UID
    for (int i = 5; i > 2; i--) {
        _uid[6-1-i]  = _uid[i];
    }
    
    snprintf(sUID, sizeof(sUID), "%02d%02d%02d",_uid[0], _uid[1], _uid[2]);

    // Assign board type, defined at compile-time
    _boardId = BOARD_ID;
    // Create device UID
    _device_uid = (char *)malloc(sizeof(char) + strlen("io-wipper-") + strlen(_boardId) + strlen(sUID));
    strcpy(_device_uid, "io-wipper-");
    strcat(_device_uid, _boardId);
    strcat(_device_uid, sUID);
    //self._device_uid = "io-wipper-{}{}".format(self._board.name, str(mac_addr))

    // Create MQTT client object
    setupMQTTClient(_device_uid);

    // Assign board type info
    // TODO: Do we still need this?
    _hw_vid = USB_VID;
    _hw_pid = USB_PID;

    // allocate memory for reserved topics
    _topic_description = (char *)malloc(sizeof(char) * strlen(_username) \
    + strlen("/wprsnpr") + strlen(TOPIC_DESCRIPTION) + strlen("status") + 1);

    // Check-in status topic
    _topic_description_status = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/wprsnpr/") + strlen(_device_uid) + strlen(TOPIC_DESCRIPTION) + \
    strlen("status") + strlen("broker") + 1);

    _topic_signal_device = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/") + strlen(_device_uid) +  strlen("/wprsnpr/") + \
    strlen(TOPIC_SIGNALS) + strlen("device") + 1);

    _topic_signal_brkr = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/") + strlen(_device_uid) +  strlen("/wprsnpr/") + \
    strlen(TOPIC_SIGNALS) + strlen("broker") + 1);

    // Build description check-in topic
    if (_topic_description) {
        strcpy(_topic_description, _username);
        strcat(_topic_description, "/wprsnpr");
        strcat(_topic_description, TOPIC_DESCRIPTION);
        strcat(_topic_description, "status");
    } else { // malloc failed
        _topic_description  = 0;
    }


    // build description status topic
    if (_topic_description_status) {
        strcpy(_topic_description_status, _username);
        strcat(_topic_description_status, "/wprsnpr/");
        strcat(_topic_description_status, _device_uid);
        strcat(_topic_description_status, TOPIC_DESCRIPTION);
        strcat(_topic_description_status, "status");
        strcat(_topic_description_status, "/broker");
    } else { // malloc failed
        _topic_description_status = 0;
    }

    // build incoming signal topic
    if (_topic_signal_device) {
        strcpy(_topic_signal_device, _username);
        strcat(_topic_signal_device, "/wprsnpr/");
        strcat(_topic_signal_device, _device_uid);
        strcat(_topic_signal_device, TOPIC_SIGNALS);
        strcat(_topic_signal_device, "device");
    } else { // malloc failed
        _topic_signal_device = 0;
    }

    // build signals outgoing topic
    if (_topic_signal_brkr) {
        strcpy(_topic_signal_brkr, _username);
        strcat(_topic_signal_brkr, "/wprsnpr/");
        strcat(_topic_signal_brkr, _device_uid);
        strcat(_topic_signal_brkr, TOPIC_SIGNALS);
        strcat(_topic_signal_brkr, "broker");
    } else { // malloc failed
        _topic_signal_brkr = 0;
    }

}

/**************************************************************************/
/*!
    @brief    Connects to Adafruit IO+ Wippersnapper broker.
*/
/**************************************************************************/
void Wippersnapper::connect() {
    // WS_DEBUG_PRINTLN("::connect()");
    _status = WS_IDLE;
    _boardStatus = WS_BOARD_DEF_IDLE;

    //WS_DEBUG_PRINTLN("Generating WS Feeds...");
    generate_feeds();

    // Subscription to listen to commands from the server
    _topic_signal_brkr_sub = new Adafruit_MQTT_Subscribe(_mqtt, _topic_signal_brkr);
    _topic_signal_brkr_sub->setCallback(cbSignalTopic);
    _mqtt->subscribe(_topic_signal_brkr_sub);

    // Publish to outgoing commands channel, server listens to this sub-topic
    _topic_signal_device_pub = new Adafruit_MQTT_Publish(_mqtt, _topic_signal_device);

    // Create a subscription to the description status response topic
    _topic_description_sub = new Adafruit_MQTT_Subscribe(_mqtt, _topic_description_status);

    // set callback and subscribe
    _topic_description_sub->setCallback(cbDescriptionStatus);
    _mqtt->subscribe(_topic_description_sub);

    // Connect network interface
    WS_DEBUG_PRINT("Connecting to WiFi...");
    _connect();
    WS_DEBUG_PRINTLN("WiFi Connected!");

    // Wait for connection to broker
    WS_DEBUG_PRINT("Connecting to Wippersnapper MQTT...");
    while (status() < WS_CONNECTED) {
        WS_DEBUG_PRINT(".");
        delay(500);
    }
    WS_DEBUG_PRINTLN("MQTT Connected!");

    // Send hardware description to broker
    if (!sendGetHardwareDescription(10)){
        // TODO: get error types back from function instead of bool, verify against resp.
        WS_DEBUG_PRINTLN("Board not identified with broker.");
        for(;;);
    }

}

/**************************************************************************/
/*!
    @brief    Disconnects from Adafruit IO+ Wippersnapper.
*/
/**************************************************************************/
void Wippersnapper::disconnect() {
    _disconnect();
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

        if (!sendGetHardwareDescription(10)){
            // TODO: get error types back from function instead of bool, verify against resp.
            return status();
        }
    }
    return status();
}

/**************************************************************************/
/*!
    @brief    Checks and handles connection to MQTT broker.
*/
/**************************************************************************/
ws_status_t Wippersnapper::checkMQTTConnection(uint32_t timeStart) {
    while(mqttStatus() != WS_CONNECTED && millis() - timeStart < WS_KEEPALIVE_INTERVAL) {
    }
    if (mqttStatus() != WS_CONNECTED) {
        return status();
    }
    return status();
}

/**************************************************************************/
/*!
    @brief    Pings MQTT broker to keep connection alive.
*/
/**************************************************************************/
void Wippersnapper::ping() {
    if (millis() > (_prv_ping + WS_KEEPALIVE_INTERVAL)) {
        _mqtt->ping();
        _prv_ping = millis();
    }
}

/**************************************************************************/
/*!
    @brief    Processes incoming commands and handles network connection.
*/
/**************************************************************************/
ws_status_t Wippersnapper::run() {
    uint32_t timeStart = millis();

    // Check network connection
    checkNetworkConnection(timeStart); // TODO: handle this better
    // Check and handle MQTT connection
    checkMQTTConnection(timeStart); // TODO: handle this better

    // Ping broker if keepalive elapsed
    ping();

    // Process all incoming packets from Wippersnapper MQTT Broker
    _mqtt->processPackets(500);

    // Handle incoming signal message
    int n; // TODO: decl. in .h instead
    n = memcmp(_buffer, _buffer_state, sizeof(_buffer));
    if (! n == 0) {
        WS_DEBUG_PRINTLN("New data in message buffer");

        // Create empty signal packet struct.
        wippersnapper_signal_v1_CreateSignalRequest decodedSignalMessage = wippersnapper_signal_v1_CreateSignalRequest_init_zero;

        // Attempt to decode a signal message packet
        if (! decodeSignalMsg(&decodedSignalMessage)) {
            WS_DEBUG_PRINTLN("ERROR: Failed to decode signal message");
            return status();
        }

        // update _buffer_state with contents of new message
        // TODO: Sizeof may not work, possibly use bufSize instead
        memcpy(_buffer_state, _buffer, sizeof(_buffer));
    }
    // Send updated pin value to broker
    if ( ws_pinInfo.pinValue != ws_pinInfo.prvPinValue ) {
        WS_DEBUG_PRINT("Pin Values: "); WS_DEBUG_PRINT(ws_pinInfo.pinValue);
        WS_DEBUG_PRINT(" "); WS_DEBUG_PRINT(ws_pinInfo.prvPinValue);
        //sendPinEvent();
        ws_pinInfo.prvPinValue = ws_pinInfo.pinValue;
    }

    return status();
}

/**************************************************************************/
/*!
    @brief    Sends board description message to Wippersnapper
*/
/**************************************************************************/
bool Wippersnapper::registerBoard() {
    WS_DEBUG_PRINT("registerBoard");

    Wippersnapper_Registration *newBoard = new Wippersnapper_Registration(this);
    newBoard->set_machine_name(_boardId);
    newBoard->set_uid(atoi(sUID));

    if (! newBoard->encode_description()){
        return false;
    }

    if (! newBoard->publish_description()) {
        return false;
    }
    return true;
}

/***************************************************************************/
/*!
    @brief    Sends board description message and verifies broker's response
*/
/***************************************************************************/
bool Wippersnapper::sendGetHardwareDescription(uint8_t retries=10){
        uint8_t retryCount = 0;

        // Publish board definition message to broker
        if (!registerBoard()) {
            _boardStatus = WS_BOARD_DEF_SEND_FAILED;
            WS_DEBUG_PRINTLN("Unable to send board description to broker");
            return false;
        }

        // Validate broker's response
        while (_boardStatus == WS_BOARD_DEF_SENT) {
            _mqtt->processPackets(500); // process messages
            delay(1000);
            retryCount++;
            if (retryCount >= retries) {
                WS_DEBUG_PRINTLN(_boardStatus);
                WS_DEBUG_PRINTLN("Unable to validate board with broker, failing out..");
                return false;
            }
        }
        if (!_boardStatus == WS_BOARD_DEF_OK) {
            return false;
        }
        return true;
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
    return _boardStatus;
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
    WS_DEBUG_PRINTLN(_mqtt->connectErrorString(_status));
    return _status;
  }

  if (_mqtt->connected())
    return WS_CONNECTED;

  // prevent fast reconnect attempts, except for the first time through
  if (_last_mqtt_connect == 0 ||
      millis() - _last_mqtt_connect > WS_KEEPALIVE_INTERVAL) {
    _last_mqtt_connect = millis();
    switch (_mqtt->connect(_username, _key)) {
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
