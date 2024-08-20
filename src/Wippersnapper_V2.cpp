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

// Decoders //

/****************************************************************************/
/*!
    @brief    Configures an analog input pin according to a
                wippersnapper_pin_v1_ConfigurePinRequest message.
    @param    pinMsg
              Pointer to a wippersnapper_pin_v1_ConfigurePinRequest message.
    @returns  True if analog pin configured successfully, False otherwise.
*/
/****************************************************************************/
bool Wippersnapper_V2::configAnalogInPinReqV2(
    wippersnapper_pin_v1_ConfigurePinRequest *pinMsg) {
  bool is_success = true;

#if defined(ARDUINO_ARCH_RP2040)
  char *pinName = pinMsg->pin_name + 1;
  int pin = atoi(pinName);
#else
  char *pinName = pinMsg->pin_name + 1;
  int pin = atoi(pinName);
#endif

  if (pinMsg->request_type ==
      wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
    WsV2._analogIOV2->initAnalogInputPin(pin, pinMsg->period, pinMsg->pull,
                                         pinMsg->analog_read_mode);

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[Pin] Reading %s every %0.2f seconds\n",
             pinMsg->pin_name, pinMsg->period);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  } else if (
      pinMsg->request_type ==
      wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
    WsV2._analogIOV2->deinitAnalogPin(pinMsg->direction, pin);

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[Pin] De-initialized pin %s\n.", pinMsg->pin_name);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  } else {
    WS_DEBUG_PRINTLN("ERROR: Could not decode analog pin request!");
    is_success = false;
  }
  return is_success;
}

/****************************************************************************/
/*!
    @brief    Configures a pin according to a
                wippersnapper_pin_v1_ConfigurePinRequest message.
    @param    pinMsg
              Pointer to a wippersnapper_pin_v1_ConfigurePinRequest message.
    @returns  True if pin configured successfully, False otherwise.
*/
/****************************************************************************/
bool Wippersnapper_V2::configureDigitalPinReqV2(
    wippersnapper_pin_v1_ConfigurePinRequest *pinMsg) {
  bool is_success = true;
  char *pinName = pinMsg->pin_name + 1;
  int pin = atoi(pinName);

  if (pinMsg->request_type ==
      wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_CREATE) {
    // Initialize GPIO pin
    WsV2._digitalGPIOV2->initDigitalPin(pinMsg->direction, pin, pinMsg->period,
                                        pinMsg->pull);
  } else if (
      pinMsg->request_type ==
      wippersnapper_pin_v1_ConfigurePinRequest_RequestType_REQUEST_TYPE_DELETE) {
    // Delete digital GPIO pin
    WsV2._digitalGPIOV2->deinitDigitalPin(pinMsg->direction, pin);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Could not decode digital pin request type");
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
bool cbDecodePinConfigMsgV2(pb_istream_t *stream, const pb_field_t *field,
                            void **arg) {
  (void)field; // marking unused parameters to avoid compiler warning
  (void)arg;   // marking unused parameters to avoid compiler warning
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbDecodePinConfigMsgV2");

  // pb_decode the stream into a pinReqMessage
  wippersnapper_pin_v1_ConfigurePinRequest pinReqMsg =
      wippersnapper_pin_v1_ConfigurePinRequest_init_zero;
  if (!ws_pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequest_fields,
                    &pinReqMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
    is_success = false;
  }

  // Decode pin configuration request msg
  if (pinReqMsg.mode == wippersnapper_pin_v1_Mode_MODE_DIGITAL) {
    is_success = WsV2.configureDigitalPinReqV2(&pinReqMsg);
  } else if (pinReqMsg.mode == wippersnapper_pin_v1_Mode_MODE_ANALOG) {
    is_success = WsV2.configAnalogInPinReqV2(&pinReqMsg);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Pin mode invalid!");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief  Decodes repeated PinEvents (digital pin write) messages.
    @param  stream
            Input stream to read from.
    @param  field
            Message descriptor, usually autogenerated.
    @param  arg
            Stores any information the decoding callback may need.
    @returns True if successfully decoded, False otherwise.
*/
/**************************************************************************/
bool cbDecodeDigitalPinWriteMsgV2(pb_istream_t *stream, const pb_field_t *field,
                                  void **arg) {
  bool is_success = true;
  (void)field; // marking unused parameters to avoid compiler warning
  (void)arg;   // marking unused parameters to avoid compiler warning
  WS_DEBUG_PRINTLN("cbDecodeDigitalPinWriteMsgV2");

  // Decode stream into a PinEvent
  wippersnapper_pin_v1_PinEvent pinEventMsg =
      wippersnapper_pin_v1_PinEvent_init_zero;
  if (!ws_pb_decode(stream, wippersnapper_pin_v1_PinEvent_fields,
                    &pinEventMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode PinEvents")
    is_success = false;
  }

  // execute callback
  char *pinName = pinEventMsg.pin_name + 1;
  WsV2._digitalGPIOV2->digitalWriteSvc(atoi(pinName),
                                       atoi(pinEventMsg.pin_value));

  return is_success;
}

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
    @returns True if successfully decoded, false otherwise.
*/
/**************************************************************************/
bool cbSignalMsgV2(pb_istream_t *stream, const pb_field_t *field, void **arg) {
  (void)arg; // marking unused parameters to avoid compiler warning
  bool is_success = true;
  WS_DEBUG_PRINTLN("cbSignalMsgV2");

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
    msg.list.funcs.decode = cbDecodePinConfigMsgV2;
    msg.list.arg = field->pData;
    // decode each ConfigurePinRequest sub-message
    if (!ws_pb_decode(stream, wippersnapper_pin_v1_ConfigurePinRequests_fields,
                      &msg)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode CreateSignalRequest")
      is_success = false;
      WsV2.pinCfgCompletedV2 = false;
    }
    // If this is the initial configuration
    if (!WsV2.pinCfgCompletedV2) {
      WS_DEBUG_PRINTLN("Initial Pin Configuration Complete!");
      WsV2.pinCfgCompletedV2 = true;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_CreateSignalRequest_pin_events_tag) {
    WS_DEBUG_PRINTLN("Signal Msg Tag: Pin Event");
    // array to store the decoded PinEvents data
    wippersnapper_pin_v1_PinEvents msg =
        wippersnapper_pin_v1_PinEvents_init_zero;
    // set up callback
    msg.list.funcs.decode = cbDecodeDigitalPinWriteMsgV2;
    msg.list.arg = field->pData;
    // decode each PinEvents sub-message
    if (!ws_pb_decode(stream, wippersnapper_pin_v1_PinEvents_fields, &msg)) {
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
        NOTE: Should be executed in-order after a new _bufferV2 is recieved.
    @param    encodedSignalMsg
              Encoded signal message.
    @return   true if successfully decoded signal message, false otherwise.
*/
/**************************************************************************/
bool Wippersnapper_V2::decodeSignalMsgV2(
    wippersnapper_signal_v1_CreateSignalRequest *encodedSignalMsg) {
  bool is_success = true;
  WS_DEBUG_PRINTLN("decodeSignalMsgV2");

  /* Set up the payload callback, which will set up the callbacks for
  each oneof payload field once the field tag is known */
  encodedSignalMsg->cb_payload.funcs.decode = cbSignalMsgV2;

  // decode the CreateSignalRequest, calls cbSignalMessage and assoc. callbacks
  pb_istream_t stream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                    encodedSignalMsg)) {
    WS_DEBUG_PRINTLN(
        "ERROR (decodeSignalMsgV2):, Could not decode CreateSignalRequest")
    is_success = false;
  }
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Called when signal topic receives a new message. Fills
                shared buffer with data from payload.
    @param    data
                Data from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbSignalTopicV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("cbSignalTopicV2: New Msg on Signal Topic");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy data to buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Empty struct for storing the signal message
  WsV2._incomingSignalMsgV2 =
      wippersnapper_signal_v1_CreateSignalRequest_init_zero;

  // Attempt to decode a signal message
  if (!WsV2.decodeSignalMsgV2(&WsV2._incomingSignalMsgV2)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to decode signal message");
  }
}

/******************************************************************************************/
/*!
    @brief    Publishes an I2C response signal message to the broker.
    @param    msgi2cResponse
              A pointer to an I2C response message typedef.
*/
/******************************************************************************************/
void publishI2CResponseV2(wippersnapper_signal_v1_I2CResponse *msgi2cResponse) {
  size_t msgSz;
  pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_I2CResponse_fields,
                      msgi2cResponse);
  WS_DEBUG_PRINT("Publishing Message: I2CResponse...");
  // TODO: Implement MQTT publish for v2
  /*
  if (!WsV2._mqttV2->publish(WsV2._topic_signal_i2c_deviceV2,
                             WsV2._buffer_outgoingV2, msgSz, 1)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to publish I2C Response!");
  } else {
    WS_DEBUG_PRINTLN("Published!");
  }
  */
}

/******************************************************************************************/
/*!
    @brief    Encodes an wippersnapper_signal_v1_I2CResponse message.
    @param    msgi2cResponse
              A pointer to an wippersnapper_signal_v1_I2CResponse.
    @return   True if encoded successfully, False otherwise.
*/
/******************************************************************************************/
bool encodeI2CResponseV2(wippersnapper_signal_v1_I2CResponse *msgi2cResponse) {
  memset(WsV2._buffer_outgoingV2, 0, sizeof(WsV2._buffer_outgoingV2));
  pb_ostream_t ostream = pb_ostream_from_buffer(
      WsV2._buffer_outgoingV2, sizeof(WsV2._buffer_outgoingV2));
  if (!ws_pb_encode(&ostream, wippersnapper_signal_v1_I2CResponse_fields,
                    msgi2cResponse)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode I2C response message!");
    return false;
  }
  return true;
}

/******************************************************************************************/
/*!
    @brief    Initializes an I2C bus component
    @param    msgInitRequest
              A pointer to an i2c bus initialization message.
    @return   True if initialized successfully, False otherwise.
*/
/******************************************************************************************/
bool initializeI2CBusV2(wippersnapper_i2c_v1_I2CBusInitRequest msgInitRequest) {
  // FUTURE TODO:we should add support for multiple i2c ports!
  if (WsV2._isI2CPort0InitV2)
    return true;
  // Initialize bus
  WsV2._i2cPort0V2 = new WipperSnapper_Component_I2C(&msgInitRequest);
  WsV2.i2cComponentsV2.push_back(WsV2._i2cPort0V2);
  WsV2._isI2CPort0InitV2 = WsV2._i2cPort0V2->isInitialized();
  return WsV2._isI2CPort0InitV2;
}

/******************************************************************************************/
/*!
    @brief    Decodes a list of I2C Device Initialization messages.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from pb_decode calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeI2CDeviceInitRequestListV2(pb_istream_t *stream,
                                        const pb_field_t *field, void **arg) {
  (void)field; // marking unused parameters to avoid compiler warning
  (void)arg;   // marking unused parameters to avoid compiler warning
  WS_DEBUG_PRINTLN("EXEC: cbDecodeI2CDeviceInitRequestListV2");
  // Decode stream into individual msgI2CDeviceInitRequest messages
  wippersnapper_i2c_v1_I2CDeviceInitRequest msgI2CDeviceInitRequest =
      wippersnapper_i2c_v1_I2CDeviceInitRequest_init_zero;
  if (!ws_pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceInitRequest_fields,
                    &msgI2CDeviceInitRequest)) {
    WS_DEBUG_PRINTLN("ERROR: Could not decode I2CDeviceInitRequest message.");
    return false;
  }

  // Create response
  wippersnapper_signal_v1_I2CResponse msgi2cResponse =
      wippersnapper_signal_v1_I2CResponse_init_zero;
  msgi2cResponse.which_payload =
      wippersnapper_signal_v1_I2CResponse_resp_i2c_device_init_tag;

  // Check I2C bus
  if (!initializeI2CBusV2(msgI2CDeviceInitRequest.i2c_bus_init_req)) {
    WS_DEBUG_PRINTLN("ERROR: Failed to initialize I2C Bus");
    msgi2cResponse.payload.resp_i2c_device_init.bus_response =
        WsV2._i2cPort0V2->getBusStatus();
    if (!encodeI2CResponseV2(&msgi2cResponse)) {
      WS_DEBUG_PRINTLN("ERROR: encoding I2C Response!");
      return false;
    }
    publishI2CResponseV2(&msgi2cResponse);
    return true;
  }

  WsV2._i2cPort0V2->initI2CDevice(&msgI2CDeviceInitRequest);

  // Fill device's address and the initialization status
  // TODO: The filling should be done within the method though?
  msgi2cResponse.payload.resp_i2c_device_init.i2c_device_address =
      msgI2CDeviceInitRequest.i2c_device_address;
  msgi2cResponse.payload.resp_i2c_device_init.bus_response =
      WsV2._i2cPort0V2->getBusStatus();

  // Encode response
  if (!encodeI2CResponseV2(&msgi2cResponse)) {
    return false;
  }

  // Publish a response for the I2C device
  publishI2CResponseV2(&msgi2cResponse);
  return true;
}

/******************************************************************************************/
/*!
    @brief    Decodes an I2C signal request message and executes the
              callback based on the message's tag. If successful,
              publishes an I2C signal response back to the broker.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeSignalRequestI2CV2(pb_istream_t *stream, const pb_field_t *field,
                                void **arg) {
  bool is_success = true;
  (void)arg; // marking unused parameter to avoid compiler warning
  WS_DEBUG_PRINTLN("cbDecodeSignalRequestI2CV2");
  // Create I2C Response
  wippersnapper_signal_v1_I2CResponse msgi2cResponse =
      wippersnapper_signal_v1_I2CResponse_init_zero;

  if (field->tag == wippersnapper_signal_v1_I2CRequest_req_i2c_scan_tag) {
    WS_DEBUG_PRINTLN("I2C Scan Request");

    // Decode I2CBusScanRequest
    wippersnapper_i2c_v1_I2CBusScanRequest msgScanReq =
        wippersnapper_i2c_v1_I2CBusScanRequest_init_zero;
    if (!ws_pb_decode(stream, wippersnapper_i2c_v1_I2CBusScanRequest_fields,
                      &msgScanReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_i2c_v1_I2CBusScanRequest");
      return false; // fail out if we can't decode the request
    }

    // Empty response message
    wippersnapper_i2c_v1_I2CBusScanResponse scanResp =
        wippersnapper_i2c_v1_I2CBusScanResponse_init_zero;

    // Check I2C bus
    if (!initializeI2CBusV2(msgScanReq.bus_init_request)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize I2C Bus");
      msgi2cResponse.payload.resp_i2c_scan.bus_response =
          WsV2._i2cPort0V2->getBusStatus();
      if (!encodeI2CResponseV2(&msgi2cResponse)) {
        WS_DEBUG_PRINTLN("ERROR: encoding I2C Response!");
        return false;
      }
      publishI2CResponseV2(&msgi2cResponse);
      return true;
    }

    // Scan I2C bus
    scanResp = WsV2._i2cPort0V2->scanAddresses();

    // Fill I2CResponse
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_scan_tag;
    memcpy(msgi2cResponse.payload.resp_i2c_scan.addresses_found,
           scanResp.addresses_found, sizeof(scanResp.addresses_found));
    msgi2cResponse.payload.resp_i2c_scan.addresses_found_count =
        scanResp.addresses_found_count;

    msgi2cResponse.payload.resp_i2c_scan.bus_response = scanResp.bus_response;
    // Encode I2CResponse
    if (!encodeI2CResponseV2(&msgi2cResponse)) {
      return false;
    }
  } else if (
      field->tag ==
      wippersnapper_signal_v1_I2CRequest_req_i2c_device_init_requests_tag) {
    WS_DEBUG_PRINTLN("I2C Device LIST Init Request Found!");

    // Decode stream
    wippersnapper_i2c_v1_I2CDeviceInitRequests msgI2CDeviceInitRequestList =
        wippersnapper_i2c_v1_I2CDeviceInitRequests_init_zero;
    // Set up callback
    msgI2CDeviceInitRequestList.list.funcs.decode =
        cbDecodeI2CDeviceInitRequestListV2;
    msgI2CDeviceInitRequestList.list.arg = field->pData;
    // Decode each sub-message
    if (!ws_pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceInitRequests_fields,
                      &msgI2CDeviceInitRequestList)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode I2CDeviceInitRequests");
      is_success = false;
    }
    // return so we don't publish an empty message, we already published within
    // cbDecodeI2CDeviceInitRequestListV2() for each device
    return is_success;
  } else if (field->tag ==
             wippersnapper_signal_v1_I2CRequest_req_i2c_device_init_tag) {
    WS_DEBUG_PRINTLN("I2C Device Init Request Found!");

    // Decode stream into an I2CDeviceInitRequest
    wippersnapper_i2c_v1_I2CDeviceInitRequest msgI2CDeviceInitRequest =
        wippersnapper_i2c_v1_I2CDeviceInitRequest_init_zero;
    // Decode stream into struct, msgI2CDeviceInitRequest
    if (!ws_pb_decode(stream, wippersnapper_i2c_v1_I2CDeviceInitRequest_fields,
                      &msgI2CDeviceInitRequest)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode I2CDeviceInitRequest message.");
      return false; // fail out if we can't decode
    }

    // Create empty response
    msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_device_init_tag;

    // Check I2C bus
    if (!initializeI2CBusV2(msgI2CDeviceInitRequest.i2c_bus_init_req)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to initialize I2C Bus");
      msgi2cResponse.payload.resp_i2c_device_init.bus_response =
          WsV2._i2cPort0V2->getBusStatus();
      if (!encodeI2CResponseV2(&msgi2cResponse)) {
        WS_DEBUG_PRINTLN("ERROR: encoding I2C Response!");
        return false;
      }
      publishI2CResponseV2(&msgi2cResponse);
      return true;
    }

    // Initialize I2C device
    WsV2._i2cPort0V2->initI2CDevice(&msgI2CDeviceInitRequest);

    // Fill device's address and bus status
    msgi2cResponse.payload.resp_i2c_device_init.i2c_device_address =
        msgI2CDeviceInitRequest.i2c_device_address;
    msgi2cResponse.payload.resp_i2c_device_init.bus_response =
        WsV2._i2cPort0V2->getBusStatus();

    // Encode response
    if (!encodeI2CResponseV2(&msgi2cResponse)) {
      return false;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_I2CRequest_req_i2c_device_update_tag) {
    WS_DEBUG_PRINTLN("=> INCOMING REQUEST: I2CDeviceUpdateRequest");

    // New I2CDeviceUpdateRequest message
    wippersnapper_i2c_v1_I2CDeviceUpdateRequest msgI2CDeviceUpdateRequest =
        wippersnapper_i2c_v1_I2CDeviceUpdateRequest_init_zero;

    // Decode stream into message
    if (!ws_pb_decode(stream,
                      wippersnapper_i2c_v1_I2CDeviceUpdateRequest_fields,
                      &msgI2CDeviceUpdateRequest)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode I2CDeviceUpdateRequest message.");
      return false; // fail out if we can't decode
    }

    // Empty I2C response to fill out
    msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_device_update_tag;

    // Update I2C device's properties
    WsV2._i2cPort0V2->updateI2CDeviceProperties(&msgI2CDeviceUpdateRequest);

    // Fill address
    msgi2cResponse.payload.resp_i2c_device_update.i2c_device_address =
        msgI2CDeviceUpdateRequest.i2c_device_address;
    msgi2cResponse.payload.resp_i2c_device_update.bus_response =
        WsV2._i2cPort0V2->getBusStatus();

    // Encode response
    if (!encodeI2CResponseV2(&msgi2cResponse)) {
      return false;
    }
  } else if (field->tag ==
             wippersnapper_signal_v1_I2CRequest_req_i2c_device_deinit_tag) {
    WS_DEBUG_PRINTLN("NEW COMMAND: I2C Device Deinit");
    // Decode stream into an I2CDeviceDeinitRequest
    wippersnapper_i2c_v1_I2CDeviceDeinitRequest msgI2CDeviceDeinitRequest =
        wippersnapper_i2c_v1_I2CDeviceDeinitRequest_init_zero;
    // Decode stream into struct, msgI2CDeviceDeinitRequest
    if (!ws_pb_decode(stream,
                      wippersnapper_i2c_v1_I2CDeviceDeinitRequest_fields,
                      &msgI2CDeviceDeinitRequest)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode I2CDeviceDeinitRequest message.");
      return false; // fail out if we can't decode
    }

    // Empty I2C response to fill out
    msgi2cResponse = wippersnapper_signal_v1_I2CResponse_init_zero;
    msgi2cResponse.which_payload =
        wippersnapper_signal_v1_I2CResponse_resp_i2c_device_deinit_tag;

    // Deinitialize I2C device
    WsV2._i2cPort0V2->deinitI2CDevice(&msgI2CDeviceDeinitRequest);
    // Fill deinit response
    msgi2cResponse.payload.resp_i2c_device_deinit.i2c_device_address =
        msgI2CDeviceDeinitRequest.i2c_device_address;
    msgi2cResponse.payload.resp_i2c_device_deinit.bus_response =
        WsV2._i2cPort0V2->getBusStatus();

    // Encode response
    if (!encodeI2CResponseV2(&msgi2cResponse)) {
      return false;
    }
  } else {
    WS_DEBUG_PRINTLN("ERROR: Undefined I2C message tag");
    return false; // fail out, we didn't encode anything to publish
  }
  // Publish the I2CResponse
  publishI2CResponseV2(&msgi2cResponse);
  return is_success;
}

/**************************************************************************/
/*!
    @brief    Called when i2c signal sub-topic receives a new message and
              attempts to decode a signal request message.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbSignalI2CReqV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: Signal-I2C]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy mqtt data into buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Zero-out existing I2C signal msg.
  WsV2.msgSignalI2CV2 = wippersnapper_signal_v1_I2CRequest_init_zero;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WsV2.msgSignalI2CV2.cb_payload.funcs.decode = cbDecodeSignalRequestI2CV2;

  // Decode I2C signal request
  pb_istream_t istream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&istream, wippersnapper_signal_v1_I2CRequest_fields,
                    &WsV2.msgSignalI2CV2))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode I2C message");
}

/******************************************************************************************/
/*!
    @brief    Decodes a servo message and dispatches to the servo component.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeServoMsgV2(pb_istream_t *stream, const pb_field_t *field,
                        void **arg) {
  WS_DEBUG_PRINTLN("Decoding Servo Message...");
  (void)arg; // marking unused parameter to avoid compiler warning
  if (field->tag == wippersnapper_signal_v1_ServoRequest_servo_attach_tag) {
    WS_DEBUG_PRINTLN("GOT: Servo Attach");
    // Attempt to decode contents of servo_attach message
    wippersnapper_servo_v1_ServoAttachRequest msgServoAttachReq =
        wippersnapper_servo_v1_ServoAttachRequest_init_zero;
    if (!ws_pb_decode(stream, wippersnapper_servo_v1_ServoAttachRequest_fields,
                      &msgServoAttachReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_servo_v1_ServoAttachRequest");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[Servo ERROR] Could not decode servo request from IO!\n");
#endif
      return false; // fail out if we can't decode the request
    }
    // execute servo attach request
    char *servoPin = msgServoAttachReq.servo_pin + 1;
    bool attached = true;
    if (!WsV2._servoComponentV2->servo_attach(
            atoi(servoPin), msgServoAttachReq.min_pulse_width,
            msgServoAttachReq.max_pulse_width, msgServoAttachReq.servo_freq)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to attach servo to pin!");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[Servo ERROR] Unable to attach servo to pin! Is it already in "
          "use?\n");
#endif
      attached = false;
    } else {
      WS_DEBUG_PRINT("ATTACHED servo w/minPulseWidth: ");
      WS_DEBUG_PRINT(msgServoAttachReq.min_pulse_width);
      WS_DEBUG_PRINT(" uS and maxPulseWidth: ");
      WS_DEBUG_PRINT(msgServoAttachReq.min_pulse_width);
      WS_DEBUG_PRINT("uS on pin: ");
      WS_DEBUG_PRINTLN(servoPin);
#ifdef USE_DISPLAY
      char buffer[100];
      snprintf(buffer, 100, "[Servo] Attached servo on pin %s\n.",
               msgServoAttachReq.servo_pin);
      WsV2._ui_helper->add_text_to_terminal(buffer);
#endif
    }

    // Create and fill a servo response message
    size_t msgSz; // message's encoded size
    wippersnapper_signal_v1_ServoResponse msgServoResp =
        wippersnapper_signal_v1_ServoResponse_init_zero;
    msgServoResp.which_payload =
        wippersnapper_signal_v1_ServoResponse_servo_attach_resp_tag;
    msgServoResp.payload.servo_attach_resp.attach_success = attached;
    strcpy(msgServoResp.payload.servo_attach_resp.servo_pin,
           msgServoAttachReq.servo_pin);

    // Encode and publish response back to broker
    memset(WsV2._buffer_outgoingV2, 0, sizeof(WsV2._buffer_outgoingV2));
    pb_ostream_t ostream = pb_ostream_from_buffer(
        WsV2._buffer_outgoingV2, sizeof(WsV2._buffer_outgoingV2));
    if (!ws_pb_encode(&ostream, wippersnapper_signal_v1_ServoResponse_fields,
                      &msgServoResp)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode servo response message!");
      return false;
    }
    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_ServoResponse_fields,
                        &msgServoResp);
    WS_DEBUG_PRINT("-> Servo Attach Response...");
    // TODO: Implement MQTT publish for v2
    // WsV2._mqttV2->publish(WsV2._topic_signal_servo_deviceV2,
    // WsV2._buffer_outgoingV2, msgSz, 1);
    WS_DEBUG_PRINTLN("Published!");
  } else if (field->tag ==
             wippersnapper_signal_v1_ServoRequest_servo_write_tag) {
    WS_DEBUG_PRINTLN("GOT: Servo Write");

    // Attempt to decode contents of servo write message
    wippersnapper_servo_v1_ServoWriteRequest msgServoWriteReq =
        wippersnapper_servo_v1_ServoWriteRequest_init_zero;

    if (!ws_pb_decode(stream, wippersnapper_servo_v1_ServoWriteRequest_fields,
                      &msgServoWriteReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_servo_v1_ServoWriteRequest");
      return false; // fail out if we can't decode the request
    }
    // execute servo write request
    char *servoPin = msgServoWriteReq.servo_pin + 1;

    WS_DEBUG_PRINT("Writing pulse width of ");
    WS_DEBUG_PRINT((int)msgServoWriteReq.pulse_width);
    WS_DEBUG_PRINT("uS to servo on pin#: ");
    WS_DEBUG_PRINTLN(servoPin);

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[Servo] Writing pulse width of %u uS to pin %s\n.",
             (int)msgServoWriteReq.pulse_width, msgServoWriteReq.servo_pin);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

    WsV2._servoComponentV2->servo_write(atoi(servoPin),
                                        (int)msgServoWriteReq.pulse_width);
  } else if (field->tag ==
             wippersnapper_signal_v1_ServoRequest_servo_detach_tag) {
    WS_DEBUG_PRINTLN("GOT: Servo Detach");

    // Attempt to decode contents of servo detach message
    wippersnapper_servo_v1_ServoDetachRequest msgServoDetachReq =
        wippersnapper_servo_v1_ServoDetachRequest_init_zero;
    if (!ws_pb_decode(stream, wippersnapper_servo_v1_ServoDetachRequest_fields,
                      &msgServoDetachReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_servo_v1_ServoDetachRequest");
      return false; // fail out if we can't decode the request
    }

    // execute servo detach request
    char *servoPin = msgServoDetachReq.servo_pin + 1;
    WS_DEBUG_PRINT("Detaching servo from pin ");
    WS_DEBUG_PRINTLN(servoPin);

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[Servo] Detaching from pin %s\n.",
             msgServoDetachReq.servo_pin);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

    WsV2._servoComponentV2->servo_detach(atoi(servoPin));
  } else {
    WS_DEBUG_PRINTLN("Unable to decode servo message type!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Called when the device recieves a new message from the
              /servo/ topic.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbServoMsgV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: Servo]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy mqtt data into buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WsV2.msgServoV2.cb_payload.funcs.decode = cbDecodeServoMsgV2;

  // Decode servo message from buffer
  pb_istream_t istream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&istream, wippersnapper_signal_v1_ServoRequest_fields,
                    &WsV2.msgServoV2))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode servo message");
}

/******************************************************************************************/
/*!
    @brief    Decodes a servo message and dispatches to the servo component.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded and executed successfully, False otherwise.
*/
/******************************************************************************************/
bool cbPWMDecodeMsgV2(pb_istream_t *stream, const pb_field_t *field,
                      void **arg) {
  WS_DEBUG_PRINTLN("Decoding PWM Message...");
  (void)arg; // marking unused parameter to avoid compiler warning
  if (field->tag == wippersnapper_signal_v1_PWMRequest_attach_request_tag) {
    WS_DEBUG_PRINTLN("GOT: PWM Pin Attach");
    // Attempt to decode contents of PWM attach message
    wippersnapper_pwm_v1_PWMAttachRequest msgPWMAttachRequest =
        wippersnapper_pwm_v1_PWMAttachRequest_init_zero;
    if (!ws_pb_decode(stream, wippersnapper_pwm_v1_PWMAttachRequest_fields,
                      &msgPWMAttachRequest)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_pwm_v1_PWMAttachRequest");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[PWM ERROR]: Could not decode pin attach request!\n");
#endif
      return false; // fail out if we can't decode the request
    }

    // execute PWM pin attach request
    char *pwmPin = msgPWMAttachRequest.pin + 1;
    bool attached = WsV2._pwmComponentV2->attach(
        atoi(pwmPin), (double)msgPWMAttachRequest.frequency,
        (uint8_t)msgPWMAttachRequest.resolution);
    if (!attached) {
      WS_DEBUG_PRINTLN("ERROR: Unable to attach PWM pin");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[PWM ERROR]: Failed to attach PWM to pin! Is this pin already in "
          "use?\n");
#endif
      attached = false;
    }

    // Create and fill the response message
    wippersnapper_signal_v1_PWMResponse msgPWMResponse =
        wippersnapper_signal_v1_PWMResponse_init_zero;
    msgPWMResponse.which_payload =
        wippersnapper_signal_v1_PWMResponse_attach_response_tag;
    msgPWMResponse.payload.attach_response.did_attach = attached;
    strcpy(msgPWMResponse.payload.attach_response.pin, msgPWMAttachRequest.pin);

    // Encode and publish response back to broker
    memset(WsV2._buffer_outgoingV2, 0, sizeof(WsV2._buffer_outgoingV2));
    pb_ostream_t ostream = pb_ostream_from_buffer(
        WsV2._buffer_outgoingV2, sizeof(WsV2._buffer_outgoingV2));
    if (!ws_pb_encode(&ostream, wippersnapper_signal_v1_PWMResponse_fields,
                      &msgPWMResponse)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode PWM response message!");
      return false;
    }
    size_t msgSz; // message's encoded size
    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_PWMResponse_fields,
                        &msgPWMResponse);
    WS_DEBUG_PRINT("PUBLISHING: PWM Attach Response...");
    // TODO: Implement MQTT publish for v2
    /*
    if (!WsV2._mqttV2->publish(WsV2._topic_signal_pwm_deviceV2,
                               WsV2._buffer_outgoingV2, msgSz, 1)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to publish PWM Attach Response!");
      return false;
    }
    */
    WS_DEBUG_PRINTLN("Published!");

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[PWM] Attached on pin %s\n.",
             msgPWMResponse.payload.attach_response.pin);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  } else if (field->tag ==
             wippersnapper_signal_v1_PWMRequest_detach_request_tag) {
    WS_DEBUG_PRINTLN("GOT: PWM Pin Detach");
    // Attempt to decode contents of PWM detach message
    wippersnapper_pwm_v1_PWMDetachRequest msgPWMDetachRequest =
        wippersnapper_pwm_v1_PWMDetachRequest_init_zero;
    if (!ws_pb_decode(stream, wippersnapper_pwm_v1_PWMDetachRequest_fields,
                      &msgPWMDetachRequest)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode wippersnapper_pwm_v1_PWMDetachRequest");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[PWM ERROR] Failed to decode pin detach request from IO!\n");
#endif
      return false; // fail out if we can't decode the request
    }
    // execute PWM pin detatch request
    char *pwmPin = msgPWMDetachRequest.pin + 1;
    WsV2._pwmComponentV2->detach(atoi(pwmPin));

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[PWM] Detached on pin %s\n.",
             msgPWMDetachRequest.pin);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  } else if (field->tag ==
             wippersnapper_signal_v1_PWMRequest_write_freq_request_tag) {
    WS_DEBUG_PRINTLN("GOT: PWM Write Tone");
    // Attempt to decode contents of PWM detach message
    wippersnapper_pwm_v1_PWMWriteFrequencyRequest msgPWMWriteFreqRequest =
        wippersnapper_pwm_v1_PWMWriteFrequencyRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_pwm_v1_PWMWriteFrequencyRequest_fields,
                      &msgPWMWriteFreqRequest)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode "
                       "wippersnapper_pwm_v1_PWMWriteFrequencyRequest");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[PWM ERROR] Failed to decode frequency write request from IO!\n");
#endif
      return false; // fail out if we can't decode the request
    }

    // execute PWM pin duty cycle write request
    char *pwmPin = msgPWMWriteFreqRequest.pin + 1;
    WS_DEBUG_PRINT("Writing frequency:  ");
    WS_DEBUG_PRINT(msgPWMWriteFreqRequest.frequency);
    WS_DEBUG_PRINT("Hz to pin ");
    WS_DEBUG_PRINTLN(atoi(pwmPin));
    WsV2._pwmComponentV2->writeTone(atoi(pwmPin),
                                    msgPWMWriteFreqRequest.frequency);

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[PWM] Writing %ld Hz to pin %s\n.",
             msgPWMWriteFreqRequest.frequency, msgPWMWriteFreqRequest.pin);
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  } else if (field->tag ==
             wippersnapper_signal_v1_PWMRequest_write_duty_request_tag) {
    WS_DEBUG_PRINTLN("GOT: PWM Write Duty Cycle");

    // Attempt to decode contents of PWM detach message
    wippersnapper_pwm_v1_PWMWriteDutyCycleRequest msgPWMWriteDutyCycleRequest =
        wippersnapper_pwm_v1_PWMWriteDutyCycleRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_pwm_v1_PWMWriteDutyCycleRequest_fields,
                      &msgPWMWriteDutyCycleRequest)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode "
                       "wippersnapper_pwm_v1_PWMWriteDutyCycleRequest");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[PWM ERROR] Failed to decode duty cycle write request from IO!\n");
#endif
      return false; // fail out if we can't decode the request
    }
    // execute PWM duty cycle write request
    char *pwmPin = msgPWMWriteDutyCycleRequest.pin + 1;
    WsV2._pwmComponentV2->writeDutyCycle(
        atoi(pwmPin), (int)msgPWMWriteDutyCycleRequest.duty_cycle);

#ifdef USE_DISPLAY
    char buffer[100];
    snprintf(buffer, 100, "[PWM] Writing duty cycle %d to pin %d\n.",
             (int)msgPWMWriteDutyCycleRequest.duty_cycle, atoi(pwmPin));
    WsV2._ui_helper->add_text_to_terminal(buffer);
#endif

  } else {
    WS_DEBUG_PRINTLN("Unable to decode PWM message type!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Called when the device recieves a new message from the
              /pwm/ topic.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbPWMMsgV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: PWM]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy mqtt data into buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WsV2.msgPWMV2.cb_payload.funcs.decode = cbPWMDecodeMsgV2;

  // Decode servo message from buffer
  pb_istream_t istream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&istream, wippersnapper_signal_v1_PWMRequest_fields,
                    &WsV2.msgPWMV2))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode PWM message");
}

/******************************************************************************************/
/*!
    @brief    Decodes a Dallas Sensor (ds18x20) signal request message and
   executes the callback based on the message's tag.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeDs18x20MsgV2(pb_istream_t *stream, const pb_field_t *field,
                          void **arg) {
  (void)arg; // marking unused parameter to avoid compiler warning
  if (field->tag ==
      wippersnapper_signal_v1_Ds18x20Request_req_ds18x20_init_tag) {
    WS_DEBUG_PRINTLN("[Message Type] Init. DS Sensor");
    // Attempt to decode contents of DS18x20 message
    wippersnapper_ds18x20_v1_Ds18x20InitRequest msgDS18xInitReq =
        wippersnapper_ds18x20_v1_Ds18x20InitRequest_init_zero;

    if (!ws_pb_decode(stream,
                      wippersnapper_ds18x20_v1_Ds18x20InitRequest_fields,
                      &msgDS18xInitReq)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode "
                       "wippersnapper_ds18x20_v1_Ds18x20InitRequest");
      return false; // fail out if we can't decode the request
    }
    WS_DEBUG_PRINT("Adding DS18x20 Component...");
    if (!WsV2._ds18x20ComponentV2->addDS18x20(&msgDS18xInitReq))
      return false;
    WS_DEBUG_PRINTLN("Added!");
  } else if (field->tag ==
             wippersnapper_signal_v1_Ds18x20Request_req_ds18x20_deinit_tag) {
    WS_DEBUG_PRINTLN("[Message Type] De-init. DS Sensor");
    // Attempt to decode contents of message
    wippersnapper_ds18x20_v1_Ds18x20DeInitRequest msgDS18xDeInitReq =
        wippersnapper_ds18x20_v1_Ds18x20DeInitRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_ds18x20_v1_Ds18x20DeInitRequest_fields,
                      &msgDS18xDeInitReq)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode "
                       "wippersnapper_ds18x20_v1_Ds18x20DeInitRequest");
      return false; // fail out if we can't decode the request
    }
    // exec. deinit request
    WsV2._ds18x20ComponentV2->deleteDS18x20(&msgDS18xDeInitReq);
  } else {
    WS_DEBUG_PRINTLN("ERROR: DS Message type not found!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Called when DallasSensor (DS) signal sub-topic receives a
              new message and attempts to decode the message.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbSignalDSReqV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: Signal-DS]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy mqtt data into buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Zero-out existing I2C signal msg.
  // WsV2.msgSignalDSV2 = wippersnapper_signal_v1_Ds18x20Request_init_zero;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WsV2.msgSignalDSV2.cb_payload.funcs.decode = cbDecodeDs18x20MsgV2;

  // Decode DS signal request
  pb_istream_t istream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&istream, wippersnapper_signal_v1_Ds18x20Request_fields,
                    &WsV2.msgSignalDSV2))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode DS message");
}

/******************************************************************************************/
/*!
    @brief    Decodes a pixel strand request message and executes the callback
   based on the message's tag.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodePixelsMsgV2(pb_istream_t *stream, const pb_field_t *field,
                         void **arg) {
  (void)arg; // marking unused parameter to avoid compiler warning
  if (field->tag ==
      wippersnapper_signal_v1_PixelsRequest_req_pixels_create_tag) {
    WS_DEBUG_PRINTLN(
        "[Message Type]: "
        "wippersnapper_signal_v1_PixelsRequest_req_pixels_create_tag");

    // attempt to decode create message
    wippersnapper_pixels_v1_PixelsCreateRequest msgPixelsCreateReq =
        wippersnapper_pixels_v1_PixelsCreateRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_pixels_v1_PixelsCreateRequest_fields,
                      &msgPixelsCreateReq)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode message of type "
                       "wippersnapper_pixels_v1_PixelsCreateRequest!");
#ifdef USE_DISPLAY
      WsV2._ui_helper->add_text_to_terminal(
          "[Pixel] Error decoding message!\n");
#endif
      return false;
    }

    // Add a new strand
    return WsV2._ws_pixelsComponentV2->addStrand(&msgPixelsCreateReq);
  } else if (field->tag ==
             wippersnapper_signal_v1_PixelsRequest_req_pixels_delete_tag) {
    WS_DEBUG_PRINTLN(
        "[Message Type]: "
        "wippersnapper_signal_v1_PixelsRequest_req_pixels_delete_tag");

    // attempt to decode delete strand message
    wippersnapper_pixels_v1_PixelsDeleteRequest msgPixelsDeleteReq =
        wippersnapper_pixels_v1_PixelsDeleteRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_pixels_v1_PixelsDeleteRequest_fields,
                      &msgPixelsDeleteReq)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode message of type "
                       "wippersnapper_pixels_v1_PixelsDeleteRequest!");
      return false;
    }

    // delete strand
    WsV2._ws_pixelsComponentV2->deleteStrand(&msgPixelsDeleteReq);
  } else if (field->tag ==
             wippersnapper_signal_v1_PixelsRequest_req_pixels_write_tag) {
    WS_DEBUG_PRINTLN(
        "[Message Type]: "
        "wippersnapper_signal_v1_PixelsRequest_req_pixels_write_tag");

    // attempt to decode pixel write message
    wippersnapper_pixels_v1_PixelsWriteRequest msgPixelsWritereq =
        wippersnapper_pixels_v1_PixelsWriteRequest_init_zero;
    if (!ws_pb_decode(stream, wippersnapper_pixels_v1_PixelsWriteRequest_fields,
                      &msgPixelsWritereq)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode message of type "
                       "wippersnapper_pixels_v1_PixelsWriteRequest!");
      return false;
    }

    // fill strand
    WsV2._ws_pixelsComponentV2->fillStrand(&msgPixelsWritereq);
  } else {
    WS_DEBUG_PRINTLN("ERROR: Pixels message type not found!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Called when the device recieves a new message from the
              /pixels/ topic.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbPixelsMsgV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE [Topic: Pixels]: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy mqtt data into buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WsV2.msgPixelsV2.cb_payload.funcs.decode = cbDecodePixelsMsgV2;

  // Decode pixel message from buffer
  pb_istream_t istream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&istream, wippersnapper_signal_v1_PixelsRequest_fields,
                    &WsV2.msgPixelsV2))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode pixel topic message");
}

/******************************************************************************************/
/*!
    @brief    Decodes a UART message and executes the callback based on the
   message's tag.
    @param    stream
              Incoming data stream from buffer.
    @param    field
              Protobuf message's tag type.
    @param    arg
              Optional arguments from decoder calling function.
    @returns  True if decoded successfully, False otherwise.
*/
/******************************************************************************************/
bool cbDecodeUARTMessageV2(pb_istream_t *stream, const pb_field_t *field,
                           void **arg) {
  if (field->tag ==
      wippersnapper_signal_v1_UARTRequest_req_uart_device_attach_tag) {
    WS_DEBUG_PRINTLN(
        "[Message Type]: "
        "wippersnapper_signal_v1_UARTRequest_req_uart_device_attach_tag");

    // attempt to decode create message
    wippersnapper_uart_v1_UARTDeviceAttachRequest msgUARTInitReq =
        wippersnapper_uart_v1_UARTDeviceAttachRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_uart_v1_UARTDeviceAttachRequest_fields,
                      &msgUARTInitReq)) {
      WS_DEBUG_PRINTLN(
          "ERROR: Could not decode message of type: UARTDeviceAttachRequest!");
      return false;
    }

    // Check if bus_info is within the message
    if (!msgUARTInitReq.has_bus_info) {
      WS_DEBUG_PRINTLN("ERROR: UART bus info not found within message!");
      return false;
    }

    // Have we previously initialized the UART bus?
    if (!WsV2._uartComponentV2->isUARTBusInitialized())
      WsV2._uartComponentV2->initUARTBus(&msgUARTInitReq); // Init. UART bus

    // Attach UART device to the bus specified in the message
    bool did_begin = WsV2._uartComponentV2->initUARTDevice(&msgUARTInitReq);

    // Create a UARTResponse message
    wippersnapper_signal_v1_UARTResponse msgUARTResponse =
        wippersnapper_signal_v1_UARTResponse_init_zero;
    msgUARTResponse.which_payload =
        wippersnapper_signal_v1_UARTResponse_resp_uart_device_attach_tag;
    msgUARTResponse.payload.resp_uart_device_attach.is_success = did_begin;
    strcpy(msgUARTResponse.payload.resp_uart_device_attach.device_id,
           msgUARTInitReq.device_id);
    memset(WsV2._buffer_outgoingV2, 0, sizeof(WsV2._buffer_outgoingV2));
    pb_ostream_t ostream = pb_ostream_from_buffer(
        WsV2._buffer_outgoingV2, sizeof(WsV2._buffer_outgoingV2));
    if (!ws_pb_encode(&ostream, wippersnapper_signal_v1_UARTResponse_fields,
                      &msgUARTResponse)) {
      WS_DEBUG_PRINTLN("ERROR: Unable to encode UART response message!");
      return false;
    }
    size_t msgSz; // message's encoded size
    pb_get_encoded_size(&msgSz, wippersnapper_signal_v1_UARTResponse_fields,
                        &msgUARTResponse);
    WS_DEBUG_PRINT("PUBLISHING: UART Attach Response...");
    // TODO: Implement MQTT publish for v2
    /*
    if (!WsV2._mqttV2->publish(WsV2._topic_signal_uart_deviceV2,
                               WsV2._buffer_outgoingV2, msgSz, 1)) {
      WS_DEBUG_PRINTLN("ERROR: Failed to publish UART Attach Response!");
      return false;
    }
    WS_DEBUG_PRINTLN("Published!");
    */

  } else if (field->tag ==
             wippersnapper_signal_v1_UARTRequest_req_uart_device_detach_tag) {
    WS_DEBUG_PRINTLN("[New Message] UART Detach");
    // attempt to decode uart detach request message
    wippersnapper_uart_v1_UARTDeviceDetachRequest msgUARTDetachReq =
        wippersnapper_uart_v1_UARTDeviceDetachRequest_init_zero;
    if (!ws_pb_decode(stream,
                      wippersnapper_uart_v1_UARTDeviceDetachRequest_fields,
                      &msgUARTDetachReq)) {
      WS_DEBUG_PRINTLN("ERROR: Could not decode message!");
      return false;
    }
    // detach UART device
    WsV2._uartComponentV2->detachUARTDevice(&msgUARTDetachReq);
    WS_DEBUG_PRINTLN("Detached uart device from bus");
  } else {
    WS_DEBUG_PRINTLN("ERROR: UART message type not found!");
    return false;
  }
  return true;
}

/**************************************************************************/
/*!
    @brief    Called when the signal UART sub-topic receives a
              new message. Performs decoding.
    @param    data
              Incoming data from MQTT broker.
    @param    len
              Length of incoming data.
*/
/**************************************************************************/
void cbSignalUARTReqV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("* NEW MESSAGE on Signal of type UART: ");
  WS_DEBUG_PRINT(len);
  WS_DEBUG_PRINTLN(" bytes.");
  // zero-out current buffer
  memset(WsV2._bufferV2, 0, sizeof(WsV2._bufferV2));
  // copy mqtt data into buffer
  memcpy(WsV2._bufferV2, data, len);
  WsV2.bufSizeV2 = len;

  // Set up the payload callback, which will set up the callbacks for
  // each oneof payload field once the field tag is known
  WsV2.msgSignalUARTV2.cb_payload.funcs.decode = cbDecodeUARTMessageV2;

  // Decode DS signal request
  pb_istream_t istream = pb_istream_from_buffer(WsV2._bufferV2, WsV2.bufSizeV2);
  if (!ws_pb_decode(&istream, wippersnapper_signal_v1_UARTRequest_fields,
                    &WsV2.msgSignalUARTV2))
    WS_DEBUG_PRINTLN("ERROR: Unable to decode UART Signal message");
}

/****************************************************************************/
/*!
    @brief    Handles MQTT messages on signal topic until timeout.
    @param    outgoingSignalMsg
                Empty signal message struct.
    @param    pinName
                Name of pin.
    @param    pinVal
                Value of pin.
    @returns  True if pinEvent message encoded successfully, false otherwise.
*/
/****************************************************************************/
bool Wippersnapper_V2::encodePinEventV2(
    wippersnapper_signal_v1_CreateSignalRequest *outgoingSignalMsg,
    uint8_t pinName, int pinVal) {
  bool is_success = true;
  outgoingSignalMsg->which_payload =
      wippersnapper_signal_v1_CreateSignalRequest_pin_event_tag;
  // fill the pin_event message
  sprintf(outgoingSignalMsg->payload.pin_event.pin_name, "D%d", pinName);
  sprintf(outgoingSignalMsg->payload.pin_event.pin_value, "%d", pinVal);

  // Encode signal message
  pb_ostream_t stream = pb_ostream_from_buffer(WsV2._buffer_outgoingV2,
                                               sizeof(WsV2._buffer_outgoingV2));
  if (!ws_pb_encode(&stream, wippersnapper_signal_v1_CreateSignalRequest_fields,
                    outgoingSignalMsg)) {
    WS_DEBUG_PRINTLN("ERROR: Unable to encode signal message");
    is_success = false;
  }

  return is_success;
}

/**************************************************************************/
/*!
    @brief    Called when broker responds to a device's publish across
              the registration topic.
    @param    data
                Data from MQTT broker.
    @param    len
                Length of data received from MQTT broker.
*/
/**************************************************************************/
void cbRegistrationStatusV2(char *data, uint16_t len) {
  // call decoder for registration response msg
  WsV2.decodeRegistrationRespV2(data, len);
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
  // Conversion to match integer UID sent by encodePubRegistrationReqV2()
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
  size_t lenTopicB2d = lenUser + lenTopicX2x + lenBoardId + 1;
  size_t lenTopicD2b = lenUser + lenTopicX2x + lenBoardId + 1;
  size_t lenTopicError = lenUser + lenTopicErrorStr + 1;
  size_t lenTopicThrottle = lenUser + lenTopicThrottleStr + 1;

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
  snprintf(WsV2._topicB2d, lenTopicB2d, "%s/ws-b2d/%s", WsV2._configV2.aio_user,
           _device_uidV2);
  WS_DEBUG_PRINT("Broker-to-device topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicB2d);
  // Subscribe to broker-to-device topic
  _subscribeB2d = new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicB2d, 1);
  WsV2._mqttV2->subscribe(_subscribeB2d);
  // TODO: Implement callback for broker-to-device signal
  // _subscribeB2d->setCallback(cbBrokerToDevice);

  // Create global device to broker topic
  // Attempt to allocate memory for the broker-to-device topic
#ifdef USE_PSRAM
  WsV2._topicD2b = (char *)ps_malloc(sizeof(char) * lenTopicD2b);
#else
  WsV2._topicD2b =
      (char *)malloc(sizeof(char) * lenTopicD2b);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicD2b == NULL)
    return false;
  // Build the broker-to-device topic
  snprintf(WsV2._topicD2b, lenTopicD2b,
           "%s/ws-d2b/%s", WsV2._configV2.aio_user, _device_uidV2);
  WS_DEBUG_PRINT("Device-to-broker topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicD2b);

  // Attempt to allocate memory for the error topic
#ifdef USE_PSRAM
  WsV2._topicError =
      (char *)ps_malloc(sizeof(char) * lenTopicError);
#else
  WsV2._topicError =
      (char *)malloc(sizeof(char) * lenTopicError);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicError == NULL)
    return false;
  // Build the error topic
  snprintf(WsV2._topicError, lenTopicError, "%s/%s",
           WsV2._configV2.aio_user, "errors");
  WS_DEBUG_PRINT("Error topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicError);
  // Subscribe to the error topic
  _subscribeError = new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicError);
  WsV2._mqttV2->subscribe(_subscribeError);
// TODO: Implement the error topic callback
// _subscribeError->setCallback(cbErrorTopic);

// Attempt to allocate memory for the error topic
#ifdef USE_PSRAM
  WsV2._topicThrottle =
      (char *)ps_malloc(sizeof(char) * lenTopicThrottle);
#else
  WsV2._topicThrottle =
      (char *)malloc(sizeof(char) * lenTopicThrottle);
#endif
  // Check if memory allocation was successful
  if (WsV2._topicThrottle == NULL)
    return false;
  // Build the throttle topic
  snprintf(WsV2._topicThrottle, lenTopicThrottle, "%s/%s",
           WsV2._configV2.aio_user, "throttle");
  WS_DEBUG_PRINT("Throttle topic: ");
  WS_DEBUG_PRINTLN(WsV2._topicThrottle);
  // Subscribe to throttle topic
  _subscribeThrottle =
      new Adafruit_MQTT_Subscribe(WsV2._mqttV2, WsV2._topicThrottle);
  WsV2._mqttV2->subscribe(_subscribeThrottle);
  // TODO: Implement the throttle topic callback
  // _subscribeThrottle->setCallback(cbThrottleTopic);

  return true;
}

/****************************************************************************/
/*!
    @brief    Encodes hardware registration request message and publishes
              the message to the Adafruit IO broker.
    @returns  True if encoded and/or published successfully, False otherwise.
*/
/****************************************************************************/
bool Wippersnapper_V2::encodePubRegistrationReqV2() {
  bool _status;

  WS_DEBUG_PRINT("Encoding registration msg...");
  // Create message object
  wippersnapper_description_v1_CreateDescriptionRequest _message =
      wippersnapper_description_v1_CreateDescriptionRequest_init_zero;

  // Set machine_name
  strcpy(_message.machine_name, WS._boardId);

  // Set MAC address
  _message.mac_addr = atoi(WS.sUID);

  // Set version
  strcpy(_message.str_version, WS_VERSION);

  // encode registration request message
  uint8_t _message_buffer[256];
  pb_ostream_t _msg_stream =
      pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

  _status = ws_pb_encode(
      &_msg_stream,
      wippersnapper_description_v1_CreateDescriptionRequest_fields, &_message);
  size_t _message_len = _msg_stream.bytes_written;

  // verify message
  if (!_status)
    return _status;

  // pubish message
  WS.publish(WS._topic_description, _message_buffer, _message_len, 1);
  WS_DEBUG_PRINTLN("Published!");
  WS._boardStatus = WS_BOARD_DEF_SENT;

  return _status;
}

/****************************************************************************/
/*!
    @brief    Polls the broker for the hardware registration response message.

              NOTE: This function is BLOCKING and will trigger a WDT reset
              if the message has not arrived.

             NOTE: The registration response msg will arrive
             async. at the `cbRegistrationStatus` function
             and set the `boardStatus`
*/
/****************************************************************************/
void Wippersnapper_V2::pollRegistrationRespV2() {
  // Blocking loop, WDT reset upon failure.
  while (WS._boardStatus != WS_BOARD_DEF_OK) {
    WS_DEBUG_PRINT("Polling for registration message response...");
    WS_DEBUG_PRINTLN(WS._boardStatus);
    statusLEDBlink(WS_LED_STATUS_WAITING_FOR_REG_MSG);
    WS._mqtt->processPackets(20); // long-poll
  }
}

/****************************************************************************/
/*!
    @brief    Decodes hardware registration response message from the
              Adafruit IO MQTT broker and initializes hardware components.
    @param    data
              MQTT message from the Adafruit IO MQTT broker.
    @param    len
              Length of data from the Adafruit IO MQTT broker.
*/
/****************************************************************************/
void Wippersnapper_V2::decodeRegistrationRespV2(char *data, uint16_t len) {
  WS_DEBUG_PRINTLN("GOT Registration Response Message:");
  uint8_t buffer[len];
  memcpy(buffer, data, len);

  // init. CreateDescriptionResponse message
  wippersnapper_description_v1_CreateDescriptionResponse message =
      wippersnapper_description_v1_CreateDescriptionResponse_init_zero;

  // create input stream for buffer
  pb_istream_t stream = pb_istream_from_buffer(buffer, len);
  // decode the stream
  if (!ws_pb_decode(
          &stream,
          wippersnapper_description_v1_CreateDescriptionResponse_fields,
          &message)) {
    WS.haltError("Could not decode registration response");
  }
  // Decode registration response message
  if (message.response ==
      wippersnapper_description_v1_CreateDescriptionResponse_Response_RESPONSE_OK) {
    WS_DEBUG_PRINTLN("Hardware Response Msg:")
    WS_DEBUG_PRINT("\tGPIO Pins: ");
    WS_DEBUG_PRINTLN(message.total_gpio_pins);
    WS_DEBUG_PRINT("\tAnalog Pins: ");
    WS_DEBUG_PRINTLN(message.total_analog_pins);
    WS_DEBUG_PRINT("\tReference voltage: ");
    WS_DEBUG_PRINT(message.reference_voltage);
    WS_DEBUG_PRINTLN("v");
    // Initialize Digital IO class
    WS._digitalGPIO = new Wippersnapper_DigitalGPIO(message.total_gpio_pins);
    // Initialize Analog IO class
    WS._analogIO = new Wippersnapper_AnalogIO(message.total_analog_pins,
                                              message.reference_voltage);
    WS._boardStatus = WS_BOARD_DEF_OK;

    // Publish RegistrationComplete message to broker
    wippersnapper_description_v1_RegistrationComplete msg =
        wippersnapper_description_v1_RegistrationComplete_init_zero;
    msg.is_complete = true;

    // encode registration request message
    uint8_t _message_buffer[128];
    pb_ostream_t _msg_stream =
        pb_ostream_from_buffer(_message_buffer, sizeof(_message_buffer));

    bool _status = ws_pb_encode(
        &_msg_stream, wippersnapper_description_v1_RegistrationComplete_fields,
        &msg);
    size_t _message_len = _msg_stream.bytes_written;

    // verify message encoded correctly
    if (!_status) {
      WS._boardStatus = WS_BOARD_DEF_INVALID;
      return;
    }

    // Publish message
    // TODO: Implement MQTT publish for v2
    // WS.publish(_topic_description_status_completeV2, _message_buffer,
    // _message_len, 1);
    WS_DEBUG_PRINTLN("Completed registration process, configuration next!");

  } else {
    WS._boardStatus = WS_BOARD_DEF_INVALID;
  }
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

/**************************************************************************/
/*!
    @brief    Attempts to register hardware with Adafruit.io WipperSnapper.
    @returns  True if successful, False otherwise.
*/
/**************************************************************************/
bool Wippersnapper_V2::registerBoardV2() {
  WS_DEBUG_PRINTLN("Registering hardware with IO...");

  // Encode and publish registration request message to broker
  runNetFSMV2();
  WsV2.feedWDTV2();
  WS_DEBUG_PRINT("Encoding registration request...");
  if (!encodePubRegistrationReqV2())
    return false;

  // Blocking, attempt to obtain broker's response message
  runNetFSMV2();
  WsV2.feedWDTV2();
  pollRegistrationRespV2();

  return true;
}

/**************************************************************************/
/*!
    @brief    Returns the board definition status
    @return   Wippersnapper_V2 board definition status
*/
/**************************************************************************/
ws_board_status_t Wippersnapper_V2::getBoardStatusV2() {
  return WsV2._boardStatusV2;
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

/********************************************************/
/*!
    @brief  Publishes a message to the Adafruit IO
            MQTT broker. Handles network connectivity.
    @param  topic
            The MQTT topic to publish to.
    @param  payload
            The payload to publish.
    @param  bLen
            The length of the payload.
    @param  qos
            The Quality of Service to publish with.
*/
/*******************************************************/
void Wippersnapper_V2::publishV2(const char *topic, uint8_t *payload,
                                 uint16_t bLen, uint8_t qos) {
  // runNetFSMV2(); // NOTE: Removed for now, causes error with virtual
  // _connectV2 method when caused with WsV2 object in another file.
  WsV2.feedWDTV2();
  if (!WsV2._mqttV2->publish(topic, payload, bLen, qos)) {
    WS_DEBUG_PRINTLN("Failed to publish MQTT message!");
  }
}

/**************************************************************/
/*!
    @brief    Prints last reset reason of ESP32
    @param    reason
              The return code of rtc_get_reset_reason(coreNum)
*/
/**************************************************************/
void print_reset_reason_v2(int reason) {
  // //
  // https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/ResetReason/ResetReason.ino
  switch (reason) {
  case 1:
    WS_DEBUG_PRINTLN("POWERON_RESET");
    break; /**<1,  Vbat power on reset*/
  case 3:
    WS_DEBUG_PRINTLN("SW_RESET");
    break; /**<3,  Software reset digital core*/
  case 4:
    WS_DEBUG_PRINTLN("OWDT_RESET");
    break; /**<4,  Legacy watch dog reset digital core*/
  case 5:
    WS_DEBUG_PRINTLN("DEEPSLEEP_RESET");
    break; /**<5,  Deep Sleep reset digital core*/
  case 6:
    WS_DEBUG_PRINTLN("SDIO_RESET");
    break; /**<6,  Reset by SLC module, reset digital core*/
  case 7:
    WS_DEBUG_PRINTLN("TG0WDT_SYS_RESET");
    break; /**<7,  Timer Group0 Watch dog reset digital core*/
  case 8:
    WS_DEBUG_PRINTLN("TG1WDT_SYS_RESET");
    break; /**<8,  Timer Group1 Watch dog reset digital core*/
  case 9:
    WS_DEBUG_PRINTLN("RTCWDT_SYS_RESET");
    break; /**<9,  RTC Watch dog Reset digital core*/
  case 10:
    WS_DEBUG_PRINTLN("INTRUSION_RESET");
    break; /**<10, Instrusion tested to reset CPU*/
  case 11:
    WS_DEBUG_PRINTLN("TGWDT_CPU_RESET");
    break; /**<11, Time Group reset CPU*/
  case 12:
    WS_DEBUG_PRINTLN("SW_CPU_RESET");
    break; /**<12, Software reset CPU*/
  case 13:
    WS_DEBUG_PRINTLN("RTCWDT_CPU_RESET");
    break; /**<13, RTC Watch dog Reset CPU*/
  case 14:
    WS_DEBUG_PRINTLN("EXT_CPU_RESET");
    break; /**<14, for APP CPU, reseted by PRO CPU*/
  case 15:
    WS_DEBUG_PRINTLN("RTCWDT_BROWN_OUT_RESET");
    break; /**<15, Reset when the vdd voltage is not stable*/
  case 16:
    WS_DEBUG_PRINTLN("RTCWDT_RTC_RESET");
    break; /**<16, RTC Watch dog reset digital core and rtc module*/
  default:
    WS_DEBUG_PRINTLN("NO_MEAN");
  }
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
  WS_DEBUG_PRINT("ESP32 CPU0 RESET REASON: ");
  print_reset_reason_v2(0);
  WS_DEBUG_PRINT("ESP32 CPU1 RESET REASON: ");
  print_reset_reason_v2(1);
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

  // Register hardware with Wippersnapper_V2
  WS_DEBUG_PRINTLN("Registering hardware with WipperSnapper...")
  if (!registerBoardV2()) {
    haltErrorV2("Unable to register with WipperSnapper.");
  }
  runNetFSMV2();
  WsV2.feedWDTV2();

// switch to monitor screen
#ifdef USE_DISPLAY
  WS_DEBUG_PRINTLN("Clearing loading screen...");
  WsV2._ui_helper->clear_scr_load();
  WS_DEBUG_PRINTLN("building monitor screen...");
  WsV2._ui_helper->build_scr_monitor();
#endif

  // Configure hardware
  WsV2.pinCfgCompletedV2 = false;
  while (!WsV2.pinCfgCompletedV2) {
    WS_DEBUG_PRINTLN(
        "Polling for message containing hardware configuration...");
    WsV2._mqttV2->processPackets(10); // poll
  }
  // Publish that we have completed the configuration workflow
  WsV2.feedWDTV2();
  runNetFSMV2();
  publishPinConfigCompleteV2();
  WS_DEBUG_PRINTLN("Hardware configured successfully!");

  statusLEDFade(GREEN, 3);
  WS_DEBUG_PRINTLN(
      "Registration and configuration complete!\nRunning application...");
}

/**************************************************************************/
/*!
    @brief    Publishes an ACK to the broker that the device has completed
              its hardware configuration.
*/
/**************************************************************************/
void Wippersnapper_V2::publishPinConfigCompleteV2() {
  // Publish that we've set up the pins and are ready to run
  wippersnapper_signal_v1_SignalResponse msg =
      wippersnapper_signal_v1_SignalResponse_init_zero;
  msg.which_payload =
      wippersnapper_signal_v1_SignalResponse_configuration_complete_tag;
  msg.payload.configuration_complete = true;

  // encode registration request message
  uint8_t _message_bufferV2[128];
  pb_ostream_t _msg_stream =
      pb_ostream_from_buffer(_message_bufferV2, sizeof(_message_bufferV2));

  bool _status = ws_pb_encode(
      &_msg_stream, wippersnapper_description_v1_RegistrationComplete_fields,
      &msg);
  size_t _message_len = _msg_stream.bytes_written;

  // verify message encoded correctly
  if (!_status)
    haltErrorV2("Could not encode, resetting...");

  // Publish message
  WS_DEBUG_PRINTLN("Publishing to pin config complete...");
  // TODO: Implement MQTT publish for v2
  // WsV2._mqttV2->publish(WsV2._topic_device_pin_config_completeV2,
  // _message_bufferV2, _message_len, 1);
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
