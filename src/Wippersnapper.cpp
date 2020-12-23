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
Timer<16U, &millis, char *> Wippersnapper::t_timer;
Wippersnapper::pinInfo Wippersnapper::ws_pinInfo;
char Wippersnapper::timerPin[3];
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

/****************************************************************************/
/*!
    @brief    ISR which reads the value of a digital pin and updates pinInfo.
*/
/****************************************************************************/
bool Wippersnapper::cbDigitalRead(char *pinName) {
  WS_DEBUG_PRINT("cbDigitalRead(");WS_DEBUG_PRINT(pinName);WS_DEBUG_PRINTLN(")");

  // Read and store pinName into struct.
  ws_pinInfo.pinValue = digitalRead((unsigned)atoi(pinName));

  // debug TODO remove
  WS_DEBUG_PRINT("Pin Values: "); WS_DEBUG_PRINT(ws_pinInfo.pinValue);
  WS_DEBUG_PRINT(" "); WS_DEBUG_PRINTLN(ws_pinInfo.prvPinValue);
  return true; // repeat every xMS
}

/**************************************************************************/
/*!
    @brief    Returns if succcessfully sent pin event to MQTT broker,
            otherwise false;
*/
/**************************************************************************/
/* bool Wippersnapper::sendPinEvent() {
  uint8_t buffer[128]; // message buffer, TODO: Make this a shared buffer
  bool status = false;

  // create output stream for buffer
  pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
  //pb_ostream_t stream = {0};

  // Create PinEventRequest message type
  pin_v1_PinEventRequest msg;

  // Encode payload
  strcpy(msg.pin_name, "D"); // TODO: hotfix for broker to identify in desc., remove
  strcat(msg.pin_name, timerPin);
  itoa(ws_pinInfo.pinValue, msg.pin_value, 10);

  // Encode PinEventRequest message
  status = encode_unionmessage(&stream, pin_v1_PinEventRequest_fields, &msg);

  if (!status)
  {
      Serial.println("Encoding failed!");
      return false;
  }

  Serial.print("Encoded size is: ");Serial.println(stream.bytes_written);

  _mqtt->publish(_topic_signal_device, buffer, stream.bytes_written, 0);
  return true;
}
 */

/**************************************************************************/
/*!
    @brief    Executes pin events from the broker
*/
/**************************************************************************/
/* // Process pin events from the broker
bool Wippersnapper::pinEvent() {
    // strip "D" or "A" from "circuitpython-style" pin_name
    char* pinName = signalMessage.payload.pin_event.pin_name + 1;
    // Set pin value
    WS_DEBUG_PRINT("Setting ")WS_DEBUG_PRINT(atoi(pinName));
    WS_DEBUG_PRINT(" to ");WS_DEBUG_PRINTLN(atoi(signalMessage.payload.pin_event.pin_value));
    digitalWrite(atoi(pinName), atoi(signalMessage.payload.pin_event.pin_value));
    return true;
} */

/**************************************************************************/
/*!
    @brief    Configures a pin's mode, direction, pull and period.
    @return   true if the pin has been successfully configured.
*/
/**************************************************************************/
bool Wippersnapper::pinConfig(wippersnapper_signal_v1_CreateSignalRequest *decodedSignalMsg) {
    bool has_executed = false;

/* 
    if (!decodePinConfigPacket(&decodedSignalMsg)) {
        WS_DEBUG_PRINTLN("ERROR: Unable to decode pinConfig message(s)");
        has_executed = false;
    } */

    //WS_DEBUG_PRINT("Pin Name: ");WS_DEBUG_PRINTLN(signalMessage.payload.pin_config.pin_name);
    //WS_DEBUG_PRINT("Mode: ");WS_DEBUG_PRINTLN(signalMessage.payload.pin_config.mode);
    //WS_DEBUG_PRINT("Direction : ");WS_DEBUG_PRINTLN(signalMessage.payload.pin_config.direction);
    //WS_DEBUG_PRINT("Pull enabled: ");WS_DEBUG_PRINTLN(signalMessage.payload.pin_config.pull);


/* 
    ws_pinInfo.PinNameFull = signalMessage.payload.pin_config.pin_name;
    // strip "D" or "A" from "circuitpython-style" pin_name
    char* pinName = signalMessage.payload.pin_config.pin_name + 1;
    ws_pinInfo.pinName = pinName;

    // TODO: Check for pullup, configure!

    // Configure pin mode and direction
    switch(signalMessage.payload.pin_config.mode) {
        case wippersnapper_pin_v1_ConfigurePinRequest_Mode_MODE_ANALOG:
            if (signalMessage.payload.pin_config.direction == wippersnapper_pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
                WS_DEBUG_PRINTLN("* Configuring Analog input pin.");
            } else {
                WS_DEBUG_PRINTLN("* Configuring Analog output pin.");
            }
            break;
        case wippersnapper_pin_v1_ConfigurePinRequest_Mode_MODE_DIGITAL:
            // TODO: _INPUT is incorrect, should be 0x0 not 0x1
            if (signalMessage.payload.pin_config.direction == 0) {
                WS_DEBUG_PRINTLN("* Configuring digital input pin");
                long timerMs = signalMessage.payload.pin_config.period;
                WS_DEBUG_PRINTLN(timerMs);
                strcpy(timerPin, pinName);
                auto task = t_timer.every(timerMs, cbDigitalRead, timerPin);
            }
            WS_DEBUG_PRINTLN("Configuring digital pin direction");
            pinMode(atoi(ws_pinInfo.pinName), signalMessage.payload.pin_config.direction);
            break;
        default:
            WS_DEBUG_PRINTLN("Unable to obtain pin configuration from message.")
            return false;
            break;
    } */
    has_executed = true; // TODO: remove, debug only!

    return has_executed;
}

/**************************************************************************/
/*!
    @brief    Executes when signal topic receives a new message and copies
                payload and payload length.
*/
/**************************************************************************/
void Wippersnapper::cbSignalTopic(char *data, uint16_t len) {
    WS_DEBUG_PRINTLN("cbSignalTopic()");
    memcpy(_buffer, data, len);
    bufSize = len;
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

    switch(field->tag) {
        case wippersnapper_signal_v1_CreateSignalRequest_pin_configs_tag:
            WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Configuration");
            // Set up decode pb_callback_t for pin configs
            break;
        case wippersnapper_signal_v1_CreateSignalRequest_pin_events_tag:
            WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Event");
            // TODO: Set up decode pb_callback_t for pin configs
            break;
        default:
            WS_DEBUG_PRINTLN("ERROR: Unexpected signal msg tag.");
            break;
    }
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
        WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
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

    WS_DEBUG_PRINT("\nSuccessfully checked in, waiting for commands...")
    // WS_DEBUG_PRINTLN(_boardStatus);
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
    _device_uid = (char *)malloc(sizeof(char) + strlen(_boardId) + strlen(sUID));
    strcpy(_device_uid, _boardId);
    strcat(_device_uid, sUID);

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
    WS_DEBUG_PRINTLN("Connected!");

    // Wait for connection to broker
    WS_DEBUG_PRINT("Connecting to Wippersnapper MQTT...");
    while (status() < WS_CONNECTED) {
        WS_DEBUG_PRINT(".");
        delay(500);
    }
    WS_DEBUG_PRINTLN("Connected!");

    // Send hardware description to broker
    if (!sendGetHardwareDescription()){
        // TODO: get error types back from function instead of bool, verify against resp.
        WS_DEBUG_PRINTLN("Hardware description process failed!");
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
        if (!sendGetHardwareDescription()){
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
    while(mqttStatus() != WS_CONNECTED && millis() - timeStart < 60000) {
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
    if (millis() > (_prv_ping + 60000)) {
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
    // increment software timer
    t_timer.tick();

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
    // TODO: Sizeof may not work, possibly use bufSize instead
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
bool Wippersnapper::sendBoardDescription() {
    WS_DEBUG_PRINT("Checking into Wippersnapper...");
    uint8_t buffer[128]; // message stored in this buffer
    size_t message_length;
    bool status;

    // initialize message description
    wippersnapper_description_v1_CreateDescriptionRequest message = wippersnapper_description_v1_CreateDescriptionRequest_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // fill message fields
    strcpy(message.machine_name, _boardId);
    message.mac_addr = atoi(sUID); // TODO: Pull from UID!

    // encode message
    status = pb_encode(&stream, wippersnapper_description_v1_CreateDescriptionRequest_fields, &message);
    message_length = stream.bytes_written;

    // verify message
    if (!status) {
        WS_DEBUG_PRINTLN("encoding description message failed!");
        //printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    // publish message
    _mqtt->publish(_topic_description, buffer, message_length, 0);
    WS_DEBUG_PRINTLN("Published board description, waiting for response!");
    _boardStatus = WS_BOARD_DEF_SENT;
    return true;
}

/***************************************************************************/
/*!
    @brief    Sends board description message and verifies broker's response
*/
/***************************************************************************/
bool Wippersnapper::sendGetHardwareDescription(){
        // Send hardware characteristics to broker
        if (!sendBoardDescription()) {
            _boardStatus = WS_BOARD_DEF_SEND_FAILED;
            WS_DEBUG_PRINTLN("Unable to send board description to broker");
            return false;
        }
        WS_DEBUG_PRINTLN("Sent check-in message to Wippersnapper!");

        // Verify broker responds OK
        WS_DEBUG_PRINTLN("Verifying board definition response")
        while (getBoardStatus() != WS_BOARD_DEF_OK) {
            WS_DEBUG_PRINT(".");
            // TODO: needs a retry+timeout loop here!!
            _mqtt->processPackets(50); // run a processing loop
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
      millis() - _last_mqtt_connect > 60000) {
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
        delay(60000);
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
