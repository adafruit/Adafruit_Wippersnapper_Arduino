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
    _topic_signals_in = 0;
    _topic_signals_out = 0;

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
    free(_topic_signals_in);
    free(_topic_signals_out);
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

    if (!pb_field_iter_begin(&iter, signal_v1_CreateSignalRequest_fields, message))
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
  BC_DEBUG_PRINT("cbDigitalRead(");BC_DEBUG_PRINT(pinName);BC_DEBUG_PRINTLN(")");

  // Read and store pinName into struct.
  ws_pinInfo.pinValue = digitalRead((unsigned)atoi(pinName));

  // debug TODO remove
  BC_DEBUG_PRINT("Pin Values: "); BC_DEBUG_PRINT(ws_pinInfo.pinValue);
  BC_DEBUG_PRINT(" "); BC_DEBUG_PRINTLN(ws_pinInfo.prvPinValue);
  return true; // repeat every xMS
}

/**************************************************************************/
/*!
    @brief    Returns if succcessfully sent pin event to MQTT broker,
            otherwise false;
*/
/**************************************************************************/
bool Wippersnapper::sendPinEvent() {
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

  _mqtt->publish(_topic_signals_in, buffer, stream.bytes_written, 0);
  return true;
}

/**************************************************************************/
/*!
    @brief    Executes pin events from the broker
*/
/**************************************************************************/
// Process pin events from the broker
bool Wippersnapper::pinEvent() {
    // strip "D" or "A" from "circuitpython-style" pin_name
    char* pinName = signalMessage.payload.pin_event.pin_name + 1;
    // Set pin value
    BC_DEBUG_PRINT("Setting ")BC_DEBUG_PRINT(atoi(pinName));
    BC_DEBUG_PRINT(" to ");BC_DEBUG_PRINTLN(atoi(signalMessage.payload.pin_event.pin_value));
    digitalWrite(atoi(pinName), atoi(signalMessage.payload.pin_event.pin_value));
    return true;
}

/**************************************************************************/
/*!
    @brief    Configures a pin's mode, direction, pull and period.
*/
/**************************************************************************/
bool Wippersnapper::pinConfig()
{
    BC_DEBUG_PRINT("Pin Name: ");BC_DEBUG_PRINTLN(signalMessage.payload.pin_config.pin_name);
    BC_DEBUG_PRINT("Mode: ");BC_DEBUG_PRINTLN(signalMessage.payload.pin_config.mode);
    BC_DEBUG_PRINT("Direction : ");BC_DEBUG_PRINTLN(signalMessage.payload.pin_config.direction);
    BC_DEBUG_PRINT("Pull enabled: ");BC_DEBUG_PRINTLN(signalMessage.payload.pin_config.pull);

    ws_pinInfo.PinNameFull = signalMessage.payload.pin_config.pin_name;
    // strip "D" or "A" from "circuitpython-style" pin_name
    char* pinName = signalMessage.payload.pin_config.pin_name + 1;
    ws_pinInfo.pinName = pinName;

    // TODO: Check for pullup, configure!

    // Configure pin mode and direction
    switch(signalMessage.payload.pin_config.mode) {
        case pin_v1_ConfigurePinRequest_Mode_MODE_ANALOG:
            if (signalMessage.payload.pin_config.direction == pin_v1_ConfigurePinRequest_Direction_DIRECTION_INPUT) {
                BC_DEBUG_PRINTLN("* Configuring Analog input pin.");
            } else {
                BC_DEBUG_PRINTLN("* Configuring Analog output pin.");
            }
            break;
        case pin_v1_ConfigurePinRequest_Mode_MODE_DIGITAL:
            // TODO: _INPUT is incorrect, should be 0x0 not 0x1
            if (signalMessage.payload.pin_config.direction == 0) {
                BC_DEBUG_PRINTLN("* Configuring digital input pin");
                long timerMs = signalMessage.payload.pin_config.period;
                BC_DEBUG_PRINTLN(timerMs);
                strcpy(timerPin, pinName);
                auto task = t_timer.every(timerMs, cbDigitalRead, timerPin);
            }
            BC_DEBUG_PRINTLN("Configuring digital pin direction");
            pinMode(atoi(ws_pinInfo.pinName), signalMessage.payload.pin_config.direction);
            break;
        default:
            BC_DEBUG_PRINTLN("Unable to obtain pin configuration from message.")
            return false;
            break;
    }
    // TODO: Replace this with a return from within the switch case's calling methods
    return true;
}

/**************************************************************************/
/*!
    @brief    Executes when signal topic receives a new message and copies
                payload and payload length.
*/
/**************************************************************************/
void Wippersnapper::cbSignalTopic(char *data, uint16_t len) {
    BC_DEBUG_PRINTLN("cbSignalTopic()");
    memcpy(_buffer, data, len);
    bufSize = len;
}

/**************************************************************************/
/*!
    @brief    Decodes a signal buffer protobuf message.
        NOTE: Should be executed in-order after a new _buffer is recieved.
*/
/**************************************************************************/
bool Wippersnapper::decodeSignalMessage() {
    // create a stream which reads from buffer
    pb_istream_t stream = pb_istream_from_buffer(_buffer, bufSize);
    // decode the message
    bool status;
    status = pb_decode(&stream, signal_v1_CreateSignalRequest_fields, &signalMessage);

    if (!status) {
        BC_DEBUG_PRINTLN("Unable to decode signal message");
        return false;
    }
    return true;
}

/**************************************************************************/
/*!
    @brief    Calls a function handler provided a signal message's payload
                type.
*/
/**************************************************************************/
bool Wippersnapper::executeSignalMessageEvent() {
    // Executes signal message event based on payload type
    switch(signalMessage.which_payload) {
        case signal_v1_CreateSignalRequest_pin_config_tag:
            Serial.println("DEBUG: Pin config callback");
            pinConfig();
            break;
        case signal_v1_CreateSignalRequest_pin_event_tag:
            Serial.println("DEBUG: Pin event callback");
            pinEvent();
            break;
        case signal_v1_CreateSignalRequest_sensor_config_tag:
            Serial.println("DEBUG: Sensor config callback");
            break;
        case signal_v1_CreateSignalRequest_sensor_event_tag:
            Serial.println("DEBUG: Sensor event callback");
            break;
        case signal_v1_CreateSignalRequest_location_request_tag:
            Serial.println("DEBUG: Location request callback");
            break;
        default:
            return false;
            break;
    }
    return true;
}

/**************************************************************************/
/*!
    @brief    Executes when the broker publishes a response to the
                client's board description message.
*/
/**************************************************************************/
void cbDescriptionStatus(char *data, uint16_t len) {
    BC_DEBUG_PRINTLN("cbDescriptionStatus()");
    uint8_t buffer[len];
    memcpy(buffer, data, len);

    // init. CreateDescriptionResponse message
    description_v1_CreateDescriptionResponse message = description_v1_CreateDescriptionResponse_init_zero;
    // create input stream for buffer
    pb_istream_t stream = pb_istream_from_buffer(buffer, len);
    // decode the stream
    bool status;
    status = pb_decode(&stream, description_v1_CreateDescriptionResponse_fields, &message);

    if (!status) {
        BC_DEBUG_PRINTLN("Error decoding description status message!");
    }

    // set board status
    switch (message.response) {
        case description_v1_CreateDescriptionResponse_Response_RESPONSE_OK:
            _boardStatus = BC_BOARD_DEF_OK;
            break;
        case description_v1_CreateDescriptionResponse_Response_RESPONSE_BOARD_NOT_FOUND:
            _boardStatus = BC_BOARD_DEF_INVALID;
            break;
        case description_v1_CreateDescriptionResponse_Response_RESPONSE_UNSPECIFIED:
            _boardStatus = BC_BOARD_DEF_UNSPECIFIED;
            break;
        default:
            _boardStatus = BC_BOARD_DEF_UNSPECIFIED;
    }

    BC_DEBUG_PRINT("Board def. response: ")
    BC_DEBUG_PRINTLN(_boardStatus);
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
    char sUID[9];
    snprintf(sUID, sizeof(sUID), "%02x%02x%02x",_uid[0], _uid[1], _uid[2]);

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
    _topic_description = (char *)malloc(sizeof(char) * strlen(_username) + strlen("/wprsnpr") + strlen(TOPIC_DESCRIPTION) + 1);

    // Check-in status topic
    _topic_description_status = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/wprsnpr/") + strlen(_device_uid) + strlen(TOPIC_DESCRIPTION) + strlen("status") + 1);

    _topic_signals_in = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/") + strlen(_device_uid) +  strlen("/wprsnpr/") + \
    strlen(TOPIC_SIGNALS) + strlen("in") + 1);

    _topic_signals_out = (char *)malloc(sizeof(char) * strlen(_username) + \
    + strlen("/") + strlen(_device_uid) +  strlen("/wprsnpr/") + \
    strlen(TOPIC_SIGNALS) + strlen("out") + 1);

    // Build description check-in topic
    if (_topic_description) {
        strcpy(_topic_description, _username);
        strcat(_topic_description, "/wprsnpr/info");
    } else { // malloc failed
        _topic_description  = 0;
    }

    // {}/wprsnpr/{}/info/status
    // build description status topic
    if (_topic_description_status) {
        strcpy(_topic_description_status, _username);
        strcat(_topic_description_status, "/wprsnpr/");
        strcat(_topic_description_status, _device_uid);
        strcat(_topic_description_status, TOPIC_DESCRIPTION);
        strcat(_topic_description_status, "status");
    } else { // malloc failed
        _topic_description_status = 0;
    }

    // build incoming signal topic
    if (_topic_signals_in) {
        strcpy(_topic_signals_in, _username);
        strcat(_topic_signals_in, "/wprsnpr/");
        strcat(_topic_signals_in, _device_uid);
        strcat(_topic_signals_in, TOPIC_SIGNALS);
        strcat(_topic_signals_in, "in");
    } else { // malloc failed
        _topic_signals_in = 0;
    }

    // build signals outgoing topic
    if (_topic_signals_out) {
        strcpy(_topic_signals_out, _username);
        strcat(_topic_signals_out, "/wprsnpr/");
        strcat(_topic_signals_out, _device_uid);
        strcat(_topic_signals_out, TOPIC_SIGNALS);
        strcat(_topic_signals_out, "out");
    } else { // malloc failed
        _topic_signals_out = 0;
    }

}

/**************************************************************************/
/*!
    @brief    Connects to Adafruit IO+ Wippersnapper broker.
*/
/**************************************************************************/
void Wippersnapper::connect() {
    BC_DEBUG_PRINTLN("::connect()");
    _status = BC_IDLE;
    _boardStatus = BC_BOARD_DEF_IDLE;

    BC_DEBUG_PRINTLN("Generating WS Feeds...");
    generate_feeds();

    // Subscription to listen to commands from the server
    _topic_signals_out_sub = new Adafruit_MQTT_Subscribe(_mqtt, _topic_signals_out);
    _topic_signals_out_sub->setCallback(cbSignalTopic);
    _mqtt->subscribe(_topic_signals_out_sub);

    // Publish to outgoing commands channel, server listens to this sub-topic
    _topic_signals_in_pub = new Adafruit_MQTT_Publish(_mqtt, _topic_signals_in);

    // Create a subscription to the description status response topic
    _topic_description_sub = new Adafruit_MQTT_Subscribe(_mqtt, _topic_description_status);

    // set callback
    _topic_description_sub->setCallback(cbDescriptionStatus);

    // subscribe
    _mqtt->subscribe(_topic_description_sub);

    BC_DEBUG_PRINT("Connecting to Wippersnapper.");

    // Connect network interface
    _connect();

    // Wait for connection to broker
    while (status() < BC_CONNECTED) {
        BC_DEBUG_PRINT(".");
        delay(500);
    }
    BC_DEBUG_PRINTLN("Connected!");

    // Send hardware description to broker
    if (!sendGetHardwareDescription()){
        // TODO: get error types back from function instead of bool, verify against resp.
        BC_DEBUG_PRINTLN("Hardware description process failed!");
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
    if (status() < BC_NET_CONNECTED) {
        BC_DEBUG_PRINTLN("connection failed, reconnecting...");
        unsigned long startRetry = millis();
        while (status() < BC_CONNECTED) { // return an error on timeout
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
    while(mqttStatus() != BC_CONNECTED && millis() - timeStart < 60000) {
    }
    if (mqttStatus() != BC_CONNECTED) {
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
        BC_DEBUG_PRINTLN("New data in message buffer");
        // Decode signal message
        if (! decodeSignalMessage()) {
            return status();
        }
        if (! executeSignalMessageEvent()) {
                BC_DEBUG_PRINTLN("Err: Event failed to execute.");
        }
        // update _buffer_state with contents of new message
        // TODO: Sizeof may not work, possibly use bufSize instead
        memcpy(_buffer_state, _buffer, sizeof(_buffer));
    }
    // Send updated pin value to broker
    if ( ws_pinInfo.pinValue != ws_pinInfo.prvPinValue ) {
        BC_DEBUG_PRINT("Pin Values: "); BC_DEBUG_PRINT(ws_pinInfo.pinValue);
        BC_DEBUG_PRINT(" "); BC_DEBUG_PRINT(ws_pinInfo.prvPinValue);
        sendPinEvent();
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
    BC_DEBUG_PRINT("Publishing board description...");
    uint8_t buffer[128]; // message stored in this buffer
    size_t message_length;
    bool status;

    // initialize message definition
    description_v1_CreateDescriptionRequest message = description_v1_CreateDescriptionRequest_init_zero;
    pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));

    // fill message fields
    strcpy(message.machine_name, _boardId);
    message.mac_addr = 0x01; // TODO: Pull from UID!

    // encode message
    status = pb_encode(&stream, description_v1_CreateDescriptionRequest_fields, &message);
    message_length = stream.bytes_written;

    // verify message
    if (!status) {
        BC_DEBUG_PRINTLN("encoding description message failed!");
        //printf("Encoding failed: %s\n", PB_GET_ERROR(&stream));
        return false;
    }

    // publish message
    _mqtt->publish(_topic_description, buffer, message_length, 0);
    BC_DEBUG_PRINTLN("Published!");
    _boardStatus = BC_BOARD_DEF_SENT;
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
            _boardStatus = BC_BOARD_DEF_SEND_FAILED;
            BC_DEBUG_PRINTLN("Unable to send board description to broker");
            return false;
        }
        BC_DEBUG_PRINTLN("Sent board description to broker!");

        // Verify broker responds OK
        BC_DEBUG_PRINTLN("Verifying board definition response")
        while (getBoardStatus() != BC_BOARD_DEF_OK) {
            BC_DEBUG_PRINT(".");
            // TODO: needs a retry+timeout loop here!!
            _mqtt->processPackets(500); // run a processing loop
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
  if (net_status != BC_NET_CONNECTED) {
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
  if (_status == BC_CONNECT_FAILED) {
    BC_DEBUG_PRINT("mqttStatus() failed to connect");
    BC_DEBUG_PRINTLN(_mqtt->connectErrorString(_status));
    return _status;
  }

  if (_mqtt->connected())
    return BC_CONNECTED;

  // prevent fast reconnect attempts, except for the first time through
  if (_last_mqtt_connect == 0 ||
      millis() - _last_mqtt_connect > 60000) {
    _last_mqtt_connect = millis();
    switch (_mqtt->connect(_username, _key)) {
    case 0:
      return BC_CONNECTED;
    case 1: // invalid mqtt protocol
    case 2: // client id rejected
    case 4: // malformed user/pass
    case 5: // unauthorized
      return BC_CONNECT_FAILED;
    case 3: // mqtt service unavailable
    case 6: // throttled
    case 7: // banned -> all MQTT bans are temporary, so eventual retry is
            // permitted
      // delay to prevent fast reconnects and fast returns (backward
      // compatibility)
        delay(60000);
      return BC_DISCONNECTED;
    default:
      return BC_DISCONNECTED;
    }
  }
  return BC_DISCONNECTED;
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
    case BC_IDLE:
        return F("Idle. Waiting for connect to be called...");
    case BC_NET_DISCONNECTED:
        return F("Network disconnected.");
    case BC_DISCONNECTED:
        return F("Disconnected from Wippersnapper.");
    // FAILURE
    case BC_NET_CONNECT_FAILED:
        return F("Network connection failed.");
    case BC_CONNECT_FAILED:
        return F("Wippersnapper connection failed.");
    case BC_FINGERPRINT_INVALID:
        return F("Wippersnapper SSL fingerprint verification failed.");
    case BC_AUTH_FAILED:
        return F("Wippersnapper authentication failed.");
    // SUCCESS
    case BC_NET_CONNECTED:
        return F("Network connected.");
    case BC_CONNECTED:
        return F("Wippersnapper connected.");
    case BC_CONNECTED_INSECURE:
        return F("Wippersnapper connected. **THIS CONNECTION IS INSECURE** SSL/TLS "
                "not supported for this platform.");
    case BC_FINGERPRINT_UNSUPPORTED:
        return F("Wippersnapper connected over SSL/TLS. Fingerprint verification "
                "unsupported.");
    case BC_FINGERPRINT_VALID:
        return F("Wippersnapper connected over SSL/TLS. Fingerprint valid.");
    default:
        return F("Unknown status code");
  }
}
